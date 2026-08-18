// =====================================================================
// my_custom.cpp - Pont Modbus RTU vers la carte Daikin FWXT (EKWHCTRL/
// EKRTCTRL), intégré directement dans openHASP via son mécanisme officiel
// de code personnalisé (HASP_USE_CUSTOM).
//
// Câblage : RS485 sur GPIO2 (TXD_EXT) / GPIO1 (RXD_EXT), 9600 bauds 8N1,
// via la puce SP3485 déjà présente sur la carte du panneau -> bornier
// A/B de la carte Daikin.
//
// Registres Daikin utilisés (confirmés par le manuel officiel
// EKWHCTRL1/EKRTCTRL1 Modbus RTU) :
//   000 (T1)   lecture  - température ambiante, x0.1
//   001 (T2)   lecture  - température eau, x0.1
//   008 (SP)   lecture  - consigne active, x0.1
//   231 (SP)   écriture - nouvelle consigne (5-40°C), x0.1
//
// NOTE HONNÊTE : ce fichier n'a jamais été compilé ni testé sur du vrai
// matériel - c'est un point de départ solide (registres et câblage
// confirmés par la doc officielle qu'on a utilisée tout au long du
// projet), mais à valider/déboguer à la compilation et au premier test,
// comme tout le reste de ce projet jusqu'ici.
// =====================================================================

#if defined(HASP_USE_CUSTOM)
#include "my_custom.h"
#include <HardwareSerial.h>

// --- Configuration ---
static const int MODBUS_TX_PIN     = 2;   // TXD_EXT
static const int MODBUS_RX_PIN     = 1;   // RXD_EXT
static const uint8_t DAIKIN_ADDR   = 1;   // adresse Modbus de la carte Daikin
                                            // (réglée sur la carte elle-même,
                                            // registre 200 / dip-switches)
static const uint32_t MODBUS_BAUD  = 9600;

static HardwareSerial modbusSerial(1); // UART1 du ESP32-S3

// --- État courant (mis à jour périodiquement) ---
static float g_temp_ambiante = NAN;
static float g_temp_eau      = NAN;
static float g_consigne      = NAN;
static bool  g_modbus_ok     = false;

// =====================================================================
// Modbus RTU minimal "fait main" (fonctions 03 lecture / 06 écriture)
// Pas de dépendance à une librairie externe - évite d'avoir à modifier
// platformio.ini pour ajouter une lib tierce.
// =====================================================================

static uint16_t modbus_crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for(uint8_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for(uint8_t i = 0; i < 8; i++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void modbus_flush_input() {
    while(modbusSerial.available()) modbusSerial.read();
}

// Lit UN registre "holding" (fonction 03). Retourne true si succès.
static bool modbus_read_register(uint8_t slave_addr, uint16_t reg, int16_t& out_value) {
    uint8_t frame[8];
    frame[0] = slave_addr;
    frame[1] = 0x03;                    // Read Holding Registers
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01;                    // 1 seul registre
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    modbus_flush_input();
    modbusSerial.write(frame, 8);
    modbusSerial.flush();

    // Attente de la réponse : adresse(1) + fonction(1) + nb octets(1) +
    // données(2) + CRC(2) = 7 octets pour une lecture d'1 registre.
    uint32_t start = millis();
    uint8_t resp[7];
    uint8_t received = 0;
    while(millis() - start < 300 && received < 7) {
        if(modbusSerial.available()) {
            resp[received++] = modbusSerial.read();
        }
    }
    if(received < 7) return false; // timeout - pas de réponse

    if(resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 2) return false;

    uint16_t resp_crc = modbus_crc16(resp, 5);
    uint16_t recv_crc = resp[5] | (resp[6] << 8);
    if(resp_crc != recv_crc) return false; // CRC invalide

    out_value = (int16_t)((resp[3] << 8) | resp[4]);
    return true;
}

// Écrit UN registre "holding" (fonction 06). Retourne true si succès.
static bool modbus_write_register(uint8_t slave_addr, uint16_t reg, int16_t value) {
    uint8_t frame[8];
    frame[0] = slave_addr;
    frame[1] = 0x06;                    // Write Single Register
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    modbus_flush_input();
    modbusSerial.write(frame, 8);
    modbusSerial.flush();

    // Un write réussi renvoie un echo de la trame envoyée (8 octets).
    uint32_t start = millis();
    uint8_t resp[8];
    uint8_t received = 0;
    while(millis() - start < 300 && received < 8) {
        if(modbusSerial.available()) {
            resp[received++] = modbusSerial.read();
        }
    }
    return received == 8;
}

// =====================================================================
// Points d'accroche appelés automatiquement par openHASP
// =====================================================================

void custom_setup() {
    modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    Serial.println(F("[DAIKIN] Port Modbus RTU initialisé (GPIO2=TX, GPIO1=RX, 9600 8N1)"));
}

void custom_loop() {
    // rien ici - tout se passe dans custom_every_5seconds(), pour ne pas
    // bloquer la boucle principale (LVGL/tactile) avec les délais Modbus
}

void custom_every_second() {
    // pas utilisé pour l'instant
}

void custom_every_5seconds() {
    int16_t raw_t1, raw_t2, raw_sp;
    bool ok = true;

    Serial.println(F("[DAIKIN] Tentative de lecture Modbus..."));

    bool ok_t1 = modbus_read_register(DAIKIN_ADDR, 0, raw_t1);   // T1 - température ambiante
    bool ok_t2 = modbus_read_register(DAIKIN_ADDR, 1, raw_t2);   // T2 - température eau
    bool ok_sp = modbus_read_register(DAIKIN_ADDR, 8, raw_sp);   // SP - consigne active
    ok = ok_t1 && ok_t2 && ok_sp;

    // Logs de debug directement dans la console série - visibles même sans
    // MQTT/Home Assistant configuré, pour vérifier que le Modbus fonctionne.
    Serial.printf("[DAIKIN] T1 (ambiante) : %s (brut=%d)\n", ok_t1 ? "OK" : "ECHEC", ok_t1 ? raw_t1 : 0);
    Serial.printf("[DAIKIN] T2 (eau)      : %s (brut=%d)\n", ok_t2 ? "OK" : "ECHEC", ok_t2 ? raw_t2 : 0);
    Serial.printf("[DAIKIN] SP (consigne) : %s (brut=%d)\n", ok_sp ? "OK" : "ECHEC", ok_sp ? raw_sp : 0);

    g_modbus_ok = ok;
    if(ok) {
        g_temp_ambiante = raw_t1 / 10.0f;
        g_temp_eau      = raw_t2 / 10.0f;
        g_consigne      = raw_sp / 10.0f;

        Serial.printf("[DAIKIN] => Ambiante=%.1f°C  Eau=%.1f°C  Consigne=%.1f°C\n",
                      g_temp_ambiante, g_temp_eau, g_consigne);

        // Publie l'état vers MQTT si configuré : hasp/<plate>/state/daikin_temp etc.
        // (si MQTT n'est pas configuré, ces appels ne font simplement rien)
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", g_temp_ambiante);
        dispatch_state_subtopic("daikin_temp", buf);

        snprintf(buf, sizeof(buf), "%.1f", g_temp_eau);
        dispatch_state_subtopic("daikin_temp_eau", buf);

        snprintf(buf, sizeof(buf), "%.1f", g_consigne);
        dispatch_state_subtopic("daikin_consigne", buf);
    } else {
        Serial.println(F("[DAIKIN] => ECHEC : aucune réponse valide de la carte (vérifier câblage A/B, alimentation, adresse Modbus)"));
        dispatch_state_subtopic("daikin_status", "erreur_modbus");
    }
}

bool custom_pin_in_use(uint8_t pin) {
    return pin == MODBUS_TX_PIN || pin == MODBUS_RX_PIN;
}

// Point d'accroche appelé PAR le noyau openHASP (hasp_dispatch.cpp) lorsqu'un
// état est publié en interne - on ne s'en sert pas nous-mêmes ici (on publie
// nos propres données via dispatch_state_subtopic() dans l'autre sens),
// mais openHASP exige que cette fonction existe quelque part dans le code
// personnalisé, sinon l'assemblage final échoue.
void custom_state_subtopic(const char* subtopic, const char* payload) {
    // rien à faire ici pour l'instant
}

// Ajoute nos valeurs au message de capteurs périodique d'openHASP
void custom_get_sensors(JsonDocument& doc) {
    JsonObject sensor = doc.createNestedObject(F("daikin"));
    sensor[F("temp_ambiante")] = g_temp_ambiante;
    sensor[F("temp_eau")]      = g_temp_eau;
    sensor[F("consigne")]      = g_consigne;
    sensor[F("modbus_ok")]     = g_modbus_ok;
}

// Reçoit les commandes MQTT entrantes, ex :
//   hasp/plate/command/daikin_setpoint   payload "21.5"
void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    if(strcmp(topic, "daikin_setpoint") == 0) {
        float nouvelle_consigne = atof(payload);
        if(nouvelle_consigne >= 5.0f && nouvelle_consigne <= 40.0f) {
            int16_t raw = (int16_t)(nouvelle_consigne * 10);
            bool ok = modbus_write_register(DAIKIN_ADDR, 231, raw);
            dispatch_state_subtopic("daikin_setpoint_ack", ok ? "ok" : "erreur");
        }
    }
}

#endif // HASP_USE_CUSTOM// =====================================================================
// my_custom.cpp - Pont Modbus RTU vers la carte Daikin FWXT (EKWHCTRL/
// EKRTCTRL), intégré directement dans openHASP via son mécanisme officiel
// de code personnalisé (HASP_USE_CUSTOM).
//
// Câblage : RS485 sur GPIO2 (TXD_EXT) / GPIO1 (RXD_EXT), 9600 bauds 8N1,
// via la puce SP3485 déjà présente sur la carte du panneau -> bornier
// A/B de la carte Daikin.
//
// Registres Daikin utilisés (confirmés par le manuel officiel
// EKWHCTRL1/EKRTCTRL1 Modbus RTU) :
//   000 (T1)   lecture  - température ambiante, x0.1
//   001 (T2)   lecture  - température eau, x0.1
//   008 (SP)   lecture  - consigne active, x0.1
//   231 (SP)   écriture - nouvelle consigne (5-40°C), x0.1
//
// NOTE HONNÊTE : ce fichier n'a jamais été compilé ni testé sur du vrai
// matériel - c'est un point de départ solide (registres et câblage
// confirmés par la doc officielle qu'on a utilisée tout au long du
// projet), mais à valider/déboguer à la compilation et au premier test,
// comme tout le reste de ce projet jusqu'ici.
// =====================================================================

#if defined(HASP_USE_CUSTOM)
#include "my_custom.h"
#include <HardwareSerial.h>

// --- Configuration ---
static const int MODBUS_TX_PIN     = 2;   // TXD_EXT
static const int MODBUS_RX_PIN     = 1;   // RXD_EXT
static const uint8_t DAIKIN_ADDR   = 1;   // adresse Modbus de la carte Daikin
                                            // (réglée sur la carte elle-même,
                                            // registre 200 / dip-switches)
static const uint32_t MODBUS_BAUD  = 9600;

static HardwareSerial modbusSerial(1); // UART1 du ESP32-S3

// --- État courant (mis à jour périodiquement) ---
static float g_temp_ambiante = NAN;
static float g_temp_eau      = NAN;
static float g_consigne      = NAN;
static bool  g_modbus_ok     = false;

// =====================================================================
// Modbus RTU minimal "fait main" (fonctions 03 lecture / 06 écriture)
// Pas de dépendance à une librairie externe - évite d'avoir à modifier
// platformio.ini pour ajouter une lib tierce.
// =====================================================================

static uint16_t modbus_crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for(uint8_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for(uint8_t i = 0; i < 8; i++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void modbus_flush_input() {
    while(modbusSerial.available()) modbusSerial.read();
}

// Lit UN registre "holding" (fonction 03). Retourne true si succès.
static bool modbus_read_register(uint8_t slave_addr, uint16_t reg, int16_t& out_value) {
    uint8_t frame[8];
    frame[0] = slave_addr;
    frame[1] = 0x03;                    // Read Holding Registers
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01;                    // 1 seul registre
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    modbus_flush_input();
    modbusSerial.write(frame, 8);
    modbusSerial.flush();

    // Attente de la réponse : adresse(1) + fonction(1) + nb octets(1) +
    // données(2) + CRC(2) = 7 octets pour une lecture d'1 registre.
    uint32_t start = millis();
    uint8_t resp[7];
    uint8_t received = 0;
    while(millis() - start < 300 && received < 7) {
        if(modbusSerial.available()) {
            resp[received++] = modbusSerial.read();
        }
    }
    if(received < 7) return false; // timeout - pas de réponse

    if(resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 2) return false;

    uint16_t resp_crc = modbus_crc16(resp, 5);
    uint16_t recv_crc = resp[5] | (resp[6] << 8);
    if(resp_crc != recv_crc) return false; // CRC invalide

    out_value = (int16_t)((resp[3] << 8) | resp[4]);
    return true;
}

// Écrit UN registre "holding" (fonction 06). Retourne true si succès.
static bool modbus_write_register(uint8_t slave_addr, uint16_t reg, int16_t value) {
    uint8_t frame[8];
    frame[0] = slave_addr;
    frame[1] = 0x06;                    // Write Single Register
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] = reg & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    modbus_flush_input();
    modbusSerial.write(frame, 8);
    modbusSerial.flush();

    // Un write réussi renvoie un echo de la trame envoyée (8 octets).
    uint32_t start = millis();
    uint8_t resp[8];
    uint8_t received = 0;
    while(millis() - start < 300 && received < 8) {
        if(modbusSerial.available()) {
            resp[received++] = modbusSerial.read();
        }
    }
    return received == 8;
}

// =====================================================================
// Points d'accroche appelés automatiquement par openHASP
// =====================================================================

void custom_setup() {
    modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    Serial.println(F("[DAIKIN] Port Modbus RTU initialisé (GPIO2=TX, GPIO1=RX, 9600 8N1)"));
}

void custom_loop() {
    // rien ici - tout se passe dans custom_every_5seconds(), pour ne pas
    // bloquer la boucle principale (LVGL/tactile) avec les délais Modbus
}

void custom_every_second() {
    // pas utilisé pour l'instant
}

void custom_every_5seconds() {
    int16_t raw_t1, raw_t2, raw_sp;
    bool ok = true;

    Serial.println(F("[DAIKIN] Tentative de lecture Modbus..."));

    bool ok_t1 = modbus_read_register(DAIKIN_ADDR, 0, raw_t1);   // T1 - température ambiante
    bool ok_t2 = modbus_read_register(DAIKIN_ADDR, 1, raw_t2);   // T2 - température eau
    bool ok_sp = modbus_read_register(DAIKIN_ADDR, 8, raw_sp);   // SP - consigne active
    ok = ok_t1 && ok_t2 && ok_sp;

    // Logs de debug directement dans la console série - visibles même sans
    // MQTT/Home Assistant configuré, pour vérifier que le Modbus fonctionne.
    Serial.printf("[DAIKIN] T1 (ambiante) : %s (brut=%d)\n", ok_t1 ? "OK" : "ECHEC", ok_t1 ? raw_t1 : 0);
    Serial.printf("[DAIKIN] T2 (eau)      : %s (brut=%d)\n", ok_t2 ? "OK" : "ECHEC", ok_t2 ? raw_t2 : 0);
    Serial.printf("[DAIKIN] SP (consigne) : %s (brut=%d)\n", ok_sp ? "OK" : "ECHEC", ok_sp ? raw_sp : 0);

    g_modbus_ok = ok;
    if(ok) {
        g_temp_ambiante = raw_t1 / 10.0f;
        g_temp_eau      = raw_t2 / 10.0f;
        g_consigne      = raw_sp / 10.0f;

        Serial.printf("[DAIKIN] => Ambiante=%.1f°C  Eau=%.1f°C  Consigne=%.1f°C\n",
                      g_temp_ambiante, g_temp_eau, g_consigne);

        // Publie l'état vers MQTT si configuré : hasp/<plate>/state/daikin_temp etc.
        // (si MQTT n'est pas configuré, ces appels ne font simplement rien)
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", g_temp_ambiante);
        dispatch_state_subtopic("daikin_temp", buf);

        snprintf(buf, sizeof(buf), "%.1f", g_temp_eau);
        dispatch_state_subtopic("daikin_temp_eau", buf);

        snprintf(buf, sizeof(buf), "%.1f", g_consigne);
        dispatch_state_subtopic("daikin_consigne", buf);
    } else {
        Serial.println(F("[DAIKIN] => ECHEC : aucune réponse valide de la carte (vérifier câblage A/B, alimentation, adresse Modbus)"));
        dispatch_state_subtopic("daikin_status", "erreur_modbus");
    }
}

bool custom_pin_in_use(uint8_t pin) {
    return pin == MODBUS_TX_PIN || pin == MODBUS_RX_PIN;
}

// Ajoute nos valeurs au message de capteurs périodique d'openHASP
void custom_get_sensors(JsonDocument& doc) {
    JsonObject sensor = doc.createNestedObject(F("daikin"));
    sensor[F("temp_ambiante")] = g_temp_ambiante;
    sensor[F("temp_eau")]      = g_temp_eau;
    sensor[F("consigne")]      = g_consigne;
    sensor[F("modbus_ok")]     = g_modbus_ok;
}

// Reçoit les commandes MQTT entrantes, ex :
//   hasp/plate/command/daikin_setpoint   payload "21.5"
void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    if(strcmp(topic, "daikin_setpoint") == 0) {
        float nouvelle_consigne = atof(payload);
        if(nouvelle_consigne >= 5.0f && nouvelle_consigne <= 40.0f) {
            int16_t raw = (int16_t)(nouvelle_consigne * 10);
            bool ok = modbus_write_register(DAIKIN_ADDR, 231, raw);
            dispatch_state_subtopic("daikin_setpoint_ack", ok ? "ok" : "erreur");
        }
    }
}

#endif // HASP_USE_CUSTOM
