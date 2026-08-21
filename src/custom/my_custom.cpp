// =====================================================================
// my_custom.cpp - Lecture du capteur température/humidité SHT20 embarqué
// sur le panneau ZX3D95CE01S-TR-4848 (Panlee), intégrée dans openHASP via
// son mécanisme officiel de code personnalisé (HASP_USE_CUSTOM).
//
// HISTORIQUE DU PROJET (pour mémoire) :
// Ce fichier contenait auparavant un pont Modbus RTU vers la carte de
// contrôle Daikin EKRTCTRL2 (bornier "+A B-", GPIO4/GPIO5 via la puce
// RS485 SP3485). Après un long diagnostic (voir historique Git de ce
// fichier), la comm avec cette carte a été abandonnée : le bus "-AB+" est
// un protocole propriétaire Daikin non documenté (confirmé par le manuel
// officiel), et même une fois cette conclusion posée, aucun signal
// électrique n'a jamais pu être confirmé en sortie du panneau malgré
// plusieurs hypothèses de GPIO testées. Le projet est passé à un pilotage
// DIY des actionneurs (moteur ventilateur + moteur pas à pas) par un ESP
// secondaire, indépendant du panneau. Tout le code Modbus a été retiré
// d'ici - ce fichier ne gère plus que l'affichage local temp/humidité.
//
// SOURCE DE CETTE IMPLÉMENTATION :
// Le fabricant du panneau fournit un firmware de démo distinct (ESP-IDF +
// QMSD UI, PAS openHASP) qui lit un capteur SHT20 (I2C, adresse 0x40) sur
// le MÊME bus I2C que le contrôleur tactile FT6336U déjà utilisé par
// openHASP (SDA=GPIO15, SCL=GPIO6 - cf board.h officiel). Cette
// implémentation reprend exactement ce câblage/protocole (voir
// TR/main/sht20_bee.c du firmware de démo fabricant), portée sur la
// librairie Arduino Wire pour s'intégrer à openHASP.
//
// IMPORTANT - à vérifier par l'utilisateur :
// Le firmware de démo du fabricant fait lui-même un scan I2C au boot et
// n'active la lecture SHT20 que si l'adresse 0x40 répond ("设备存在" /
// "device present" dans leurs logs) - sous-entendu : la puce SHT20 n'est
// peut-être pas montée sur toutes les variantes de ce panneau (le zip
// fabricant s'appelle explicitement "...带温湿度源码" = "...AVEC capteur
// temp/humidité", ce qui suggère une variante spécifique). Ce fichier fait
// la même vérification au démarrage (custom_setup()) et se met en état
// "capteur absent" proprement si l'adresse 0x40 ne répond pas, plutôt que
// de bloquer. Si "SHT20 non détecté" apparaît dans les logs, vérifier
// physiquement la présence d'un petit composant (SOIC/DFN) à proximité du
// connecteur tactile, ou confirmer via un scan I2C manuel.
//
// PROTOCOLE SHT20 (repris du firmware fabricant, cf sht20_bee.c) :
//   Écrire 0xF3 (mesure T, mode "no hold") puis attendre ~85ms puis lire
//   3 octets (MSB, LSB, CRC - CRC non vérifié ici, comme dans le code
//   fabricant d'origine).
//   Écrire 0xF5 (mesure RH, mode "no hold") puis attendre ~29ms puis lire
//   3 octets de la même façon.
//   T(°C)  = 175.72 * raw16/65536 - 46.85   (raw16 = MSB<<8 | (LSB & 0xFC))
//   RH(%)  = 125.0  * raw16/65536 - 6.0
// =====================================================================

#include "my_custom.h"
#include <Wire.h>

// --- Configuration I2C (identique au bus tactile FT6336U déjà initialisé
// par le driver tactile officiel d'openHASP - on réutilise le même bus,
// pas de nouveau câblage nécessaire) ---
static const int SHT20_SDA_PIN   = 15;
static const int SHT20_SCL_PIN   = 6;
static const uint32_t I2C_FREQ   = 400000;
static const uint8_t  SHT20_ADDR = 0x40;

static const uint8_t SHT20_CMD_TEMP_NOHOLD = 0xF3;
static const uint8_t SHT20_CMD_HUM_NOHOLD  = 0xF5;

// --- État courant (mis à jour périodiquement) ---
static float g_temperature   = NAN;
static float g_humidite      = NAN;
static bool  g_sht20_present = false;

// =====================================================================
// Driver SHT20 minimal (I2C via Wire, mode "no hold" avec délai fixe -
// évite de dépendre du clock-stretching matériel pour le mode "hold")
// =====================================================================

// Vérifie la présence du capteur à l'adresse 0x40 (simple ACK I2C).
static bool sht20_probe() {
    Wire.beginTransmission(SHT20_ADDR);
    return (Wire.endTransmission() == 0);
}

// Lance une mesure (commande "no hold") et lit le résultat brut 16 bits
// après le délai de conversion requis. Retourne true si succès (ACK reçu
// à l'écriture ET aux 2 octets de données lus).
static bool sht20_read_raw(uint8_t cmd, uint16_t delay_ms, uint16_t& raw_out) {
    Wire.beginTransmission(SHT20_ADDR);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) return false;

    delay(delay_ms);

    uint8_t received = Wire.requestFrom((int)SHT20_ADDR, 3);
    if (received < 2) return false; // pas assez d'octets reçus

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    if (received >= 3) Wire.read(); // octet CRC - lu mais non vérifié

    raw_out = ((uint16_t)msb << 8) | (lsb & 0xFC); // 2 bits de statut à ignorer
    return true;
}

static bool sht20_get_temperature(float& out_c) {
    uint16_t raw;
    if (!sht20_read_raw(SHT20_CMD_TEMP_NOHOLD, 85, raw)) return false;
    out_c = 175.72f * ((float)raw / 65536.0f) - 46.85f;
    return true;
}

static bool sht20_get_humidity(float& out_rh) {
    uint16_t raw;
    if (!sht20_read_raw(SHT20_CMD_HUM_NOHOLD, 29, raw)) return false;
    out_rh = 125.0f * ((float)raw / 65536.0f) - 6.0f;
    return true;
}

// =====================================================================
// Points d'accroche appelés automatiquement par openHASP
// =====================================================================

void custom_setup() {
    // Le bus I2C (SDA=15/SCL=6) est déjà initialisé par le driver tactile
    // FT6336U d'openHASP avant l'appel à custom_setup(). On rappelle
    // Wire.begin() avec les mêmes paramètres par sécurité/portabilité -
    // sans effet de bord attendu puisque ce sont exactement les mêmes
    // broches/fréquence que celles déjà utilisées pour le tactile.
    Wire.begin(SHT20_SDA_PIN, SHT20_SCL_PIN, I2C_FREQ);

    g_sht20_present = sht20_probe();
    if (g_sht20_present) {
        Serial.println(F("[SHT20] Capteur détecté à l'adresse 0x40 (bus I2C tactile partagé)"));
    } else {
        Serial.println(F("[SHT20] AUCUN capteur détecté à l'adresse 0x40 - vérifier que la puce est bien montée sur ce panneau"));
    }
}

void custom_loop() {
    // rien ici - la lecture se fait dans custom_every_5seconds()
}

void custom_every_second() {
    // pas utilisé pour l'instant
}

void custom_every_5seconds() {
    if (!g_sht20_present) {
        // Nouvelle tentative périodique, au cas où le capteur n'était pas
        // encore prêt au boot (faux négatif) - sans bloquer le reste.
        g_sht20_present = sht20_probe();
        if (!g_sht20_present) {
            g_temperature = NAN;
            g_humidite    = NAN;
            return;
        }
        Serial.println(F("[SHT20] Capteur détecté (nouvelle tentative)"));
    }

    float t, h;
    bool ok_t = sht20_get_temperature(t);
    bool ok_h = sht20_get_humidity(h);

    if (ok_t) g_temperature = t;
    if (ok_h) g_humidite    = h;

    if (ok_t && ok_h) {
        Serial.printf("[SHT20] Température=%.1f°C  Humidité=%.1f%%\n", g_temperature, g_humidite);

        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", g_temperature);
        dispatch_state_subtopic("temperature", buf);

        snprintf(buf, sizeof(buf), "%.1f", g_humidite);
        dispatch_state_subtopic("humidite", buf);
    } else {
        Serial.printf("[SHT20] Lecture échouée (temp=%s, hum=%s) - le capteur ne répond plus\n",
                      ok_t ? "OK" : "ECHEC", ok_h ? "OK" : "ECHEC");
        g_sht20_present = false; // on retentera un probe complet au prochain cycle
    }
}

bool custom_pin_in_use(uint8_t pin) {
    // GPIO15/GPIO6 sont déjà déclarées utilisées par le driver tactile
    // officiel d'openHASP (bus I2C partagé) - pas besoin de les
    // re-déclarer ici, on ne fait que réutiliser un bus déjà géré.
    return false;
}

// Point d'accroche appelé PAR le noyau openHASP lorsqu'un état est publié
// en interne - non utilisé ici (openHASP exige juste que la fonction
// existe quelque part dans le code personnalisé).
void custom_state_subtopic(const char* subtopic, const char* payload) {
    // rien à faire ici pour l'instant
}

// Ajoute nos valeurs au message de capteurs périodique d'openHASP
void custom_get_sensors(JsonDocument& doc) {
    JsonObject sensor = doc.createNestedObject(F("sht20"));
    sensor[F("temperature")] = g_temperature;
    sensor[F("humidite")]    = g_humidite;
    sensor[F("present")]     = g_sht20_present;
}

// Réservé pour de futures commandes MQTT entrantes - rien pour l'instant
// côté température/humidité (lecture seule), mais la fonction doit exister.
void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    (void)topic;
    (void)payload;
    (void)source;
}
