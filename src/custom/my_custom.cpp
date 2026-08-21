// =====================================================================
// my_custom.cpp - Pont Modbus RTU vers la carte Daikin FWXT (EKWHCTRL/
// EKRTCTRL), intégré directement dans openHASP via son mécanisme officiel
// de code personnalisé (HASP_USE_CUSTOM).
//
// Câblage : RS485 sur GPIO4 (TX) / GPIO5 (RX), 9600 bauds 8N1, via la puce
// SP3485 déjà présente sur la carte du panneau (transceiver à direction
// automatique piloté par transistor Q3, pas de DE/RE à piloter en
// logiciel) -> bornier A/B de la carte Daikin.
//
// HISTORIQUE DES CORRECTIFS DE PINS (ne pas repartir dans une nouvelle
// hypothèse de GPIO sans raison nouvelle - voir plus bas) :
//   1) GPIO2/GPIO1 (board.h openHASP, positions module 39/40 selon la page
//      vendeur) : config d'origine, jamais testée précisément à l'oscillo
//      pour la présence d'un signal TX à l'époque.
//   2) GPIO4/GPIO5 (lecture de l'encadré "IO MAP" du schéma officiel
//      fabricant, net-labels TXD_IO/RXD_IO) : testée, AUCUN signal à
//      l'oscillo sur A/B pendant l'émission.
//   3) GPIO43/GPIO44 (pins natives "TXD"/"RXD" du module, position 33/34) :
//      testée, AUCUN signal à l'oscillo sur A/B non plus.
//   RETOUR À GPIO4/GPIO5, confirmé cette fois par INSPECTION PHYSIQUE
//   DIRECTE du circuit imprimé par l'utilisateur (traçage des pistes au
//   plus près de la puce SP3485, pas juste une lecture de schéma) - preuve
//   la plus fiable obtenue jusqu'ici. Étant donné que GPIO4/GPIO5 avait
//   déjà été testé électriquement à vide, le problème n'est donc
//   probablement PAS le choix du GPIO mais un souci en aval, au niveau de
//   la puce SP3485 elle-même ou du contrôle de direction DE/RE - à sonder
//   directement sur les pattes du composant (VCC/GND, nœud DE/RE, DI)
//   plutôt que de continuer à changer de pins ESP32.
//
// Registres Daikin utilisés (confirmés par le manuel officiel
// EKWHCTRL1/EKRTCTRL1 Modbus RTU) :
//   000 (T1)   lecture  - température ambiante, x0.1
//   001 (T2)   lecture  - température eau, x0.1
//   008 (SP)   lecture  - consigne active, x0.1
//   231 (SP)   écriture - nouvelle consigne (5-40°C), x0.1
//
// NOTE : l'adresse Modbus réelle de la carte Daikin n'étant pas confirmée
// avec certitude (dip-switches non documentés dans le manuel d'installation
// général), ce fichier scanne automatiquement une plage d'adresses
// candidates (1-16) jusqu'à obtenir une réponse valide, plutôt que de
// supposer une adresse fixe. Un dump hexadécimal de chaque trame envoyée
// et reçue (même partielle/invalide) est loggué pour faciliter le
// diagnostic bas niveau (protocole ASCII vs RTU, câblage, etc.).
// =====================================================================

#include "my_custom.h"
#include <HardwareSerial.h>

// --- Configuration ---
static const int MODBUS_TX_PIN     = 4;   // confirmé par inspection physique du PCB
static const int MODBUS_RX_PIN     = 5;   // confirmé par inspection physique du PCB
static const uint32_t MODBUS_BAUD  = 9600;

// Adresses Modbus candidates à scanner tant que l'adresse réelle n'a pas
// été trouvée. Plage large (couvre la plupart des configs dip-switch
// typiques) - à étendre si rien ne répond dans cette plage.
static const uint8_t ADDR_CANDIDATES[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
static const uint8_t NB_CANDIDATES = sizeof(ADDR_CANDIDATES) / sizeof(ADDR_CANDIDATES[0]);

static HardwareSerial modbusSerial(1); // UART1 du ESP32-S3

// --- État courant (mis à jour périodiquement) ---
static float   g_temp_ambiante = NAN;
static float   g_temp_eau      = NAN;
static float   g_consigne      = NAN;
static bool    g_modbus_ok     = false;

static uint8_t g_scan_index    = 0;
static uint8_t g_daikin_addr   = 0;      // 0 = pas encore trouvée
static bool    g_addr_found    = false;

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

static void modbus_dump(const char* label, const uint8_t* data, uint8_t len) {
    Serial.printf("[DAIKIN][%s] %d octet(s) : ", label, len);
    for(uint8_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
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
    modbus_dump("TX", frame, 8);

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
    modbus_dump("RX", resp, received);

    if(received < 7) return false; // timeout - pas de réponse (partielle ou nulle)

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
    modbus_dump("TX", frame, 8);

    // Un write réussi renvoie un echo de la trame envoyée (8 octets).
    uint32_t start = millis();
    uint8_t resp[8];
    uint8_t received = 0;
    while(millis() - start < 300 && received < 8) {
        if(modbusSerial.available()) {
            resp[received++] = modbusSerial.read();
        }
    }
    modbus_dump("RX", resp, received);

    return received == 8;
}

// =====================================================================
// Points d'accroche appelés automatiquement par openHASP
// =====================================================================

void custom_setup() {
    modbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    Serial.println(F("[DAIKIN] Port Modbus RTU initialisé (GPIO4=TX, GPIO5=RX, 9600 8N1)"));
    Serial.println(F("[DAIKIN] Démarrage du scan d'adresses Modbus (1-16)..."));
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

    if (!g_addr_found) {
        // --- Mode scan : on teste une adresse candidate par cycle ---
        uint8_t candidate = ADDR_CANDIDATES[g_scan_index];
        Serial.printf("[DAIKIN] Scan adresse Modbus : test @%d...\n", candidate);

        bool ok = modbus_read_register(candidate, 0, raw_t1); // T1 comme sonde de test
        if (ok) {
            g_daikin_addr = candidate;
            g_addr_found  = true;
            Serial.printf("[DAIKIN] *** ADRESSE TROUVEE : %d (T1 brut=%d) ***\n", candidate, raw_t1);
        } else {
            Serial.printf("[DAIKIN] Adresse %d : pas de réponse valide\n", candidate);
            g_scan_index = (g_scan_index + 1) % NB_CANDIDATES;
        }
        return; // on ne fait rien d'autre tant que l'adresse n'est pas confirmée
    }

    // --- Mode normal : adresse connue, lecture des 3 registres ---
    Serial.println(F("[DAIKIN] Tentative de lecture Modbus..."));

    bool ok_t1 = modbus_read_register(g_daikin_addr, 0, raw_t1);   // T1 - température ambiante
    bool ok_t2 = modbus_read_register(g_daikin_addr, 1, raw_t2);   // T2 - température eau
    bool ok_sp = modbus_read_register(g_daikin_addr, 8, raw_sp);   // SP - consigne active
    bool ok = ok_t1 && ok_t2 && ok_sp;

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
        // Perte de communication après avoir eu l'adresse - on relance le scan
        Serial.println(F("[DAIKIN] => ECHEC : perte de communication - retour en mode scan"));
        g_addr_found = false;
        g_scan_index = 0;
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
    sensor[F("adresse")]       = g_daikin_addr;
}

// Reçoit les commandes MQTT entrantes, ex :
//   hasp/plate/command/daikin_setpoint   payload "21.5"
void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    if(strcmp(topic, "daikin_setpoint") == 0) {
        if (!g_addr_found) {
            dispatch_state_subtopic("daikin_setpoint_ack", "erreur_adresse_inconnue");
            return;
        }
        float nouvelle_consigne = atof(payload);
        if(nouvelle_consigne >= 5.0f && nouvelle_consigne <= 40.0f) {
            int16_t raw = (int16_t)(nouvelle_consigne * 10);
            bool ok = modbus_write_register(g_daikin_addr, 231, raw);
            dispatch_state_subtopic("daikin_setpoint_ack", ok ? "ok" : "erreur");
        }
    }
}
