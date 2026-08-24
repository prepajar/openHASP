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
#include <Preferences.h>
#include <WiFi.h>
#include <string.h>
#include <math.h>

// =====================================================================
// MODE SIMULATION - premier test du "skin" openHASP sur le vrai panneau,
// sans ESP32 secondaire ni liaison CAN branchés. Température/humidité
// restent RÉELLES (capteur SHT20 déjà confirmé fonctionnel ci-dessous) ;
// vitesse ventilo et programme de flux d'air sont des valeurs FICTIVES qui
// varient toutes seules pour vérifier que l'écran affiche bien du contenu
// dynamique. À retirer/remplacer par les vraies valeurs reçues en CAN une
// fois l'ESP32 secondaire câblé (cf mémoire du projet).
// =====================================================================
#define SIMULATION_MODE 1

// --- Adressage des objets du dashboard (pages.jsonl livré à part) ---
// p1b2=humidité, p1b3=température, p1b5=consigne, p1b9=vitesse simulée,
// p1b11=programme simulé. Page 2 = confirmation réinitialisation WiFi.
// À AJUSTER ICI si tu changes les id dans pages.jsonl.

// --- Consigne utilisateur (stockée en NVS pour survivre à un reboot) ---
static Preferences prefs;
static float g_consigne = 20.0f;
static const float CONSIGNE_MIN = 5.0f;
static const float CONSIGNE_MAX = 30.0f;
static const float CONSIGNE_STEP = 0.5f;

// --- Valeurs simulées (mode simulation uniquement) ---
static float   g_sim_vitesse = 0.0f;
static uint8_t g_sim_prog_index = 0;
static const char* SIM_PROGRAMMES[] = { "Horizontal", "Oscillant", "Vertical" };
static const uint8_t SIM_PROGRAMMES_COUNT = 3;

// --- Configuration I2C (identique au bus tactile FT6336U déjà initialisé
// par le driver tactile officiel d'openHASP - on réutilise le même bus,
// pas de nouveau câblage nécessaire) ---
static const int SHT20_SDA_PIN   = 15;
static const int SHT20_SCL_PIN   = 6;
static const uint32_t I2C_FREQ   = 400000;
static const uint8_t  SHT20_ADDR = 0x40;

static const uint8_t SHT20_CMD_TEMP_NOHOLD = 0xF3;
static const uint8_t SHT20_CMD_HUM_NOHOLD  = 0xF5;

// Correction d'auto-échauffement : le capteur est sur le même PCB que le
// rétroéclairage/ESP32/WiFi, il lit donc plus chaud que l'air ambiant une
// fois le panneau chaud (observé sur ESPlogs 9 : dérive continue à la mise
// sous tension). À calibrer : une fois le panneau stabilisé (15-20 min),
// comparer avec un thermomètre de référence et ajuster cette constante
// (ex. si le capteur affiche 31.5°C pour une vraie ambiante à 29.0°C,
// mettre -2.5f). Laissé à 0 tant que la mesure de référence n'est pas faite.
static const float TEMP_CALIBRATION_OFFSET = 0.0f;

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

// CRC8 Sensirion (poly 0x31, init 0x00) - vérifie l'intégrité des 2 octets
// de donnée reçus contre le 3e octet renvoyé par le capteur. Rejette les
// lectures corrompues par un glitch I2C plutôt que de les afficher.
static uint8_t sht20_crc8(uint8_t msb, uint8_t lsb) {
    uint8_t data[2] = { msb, lsb };
    uint8_t crc = 0x00;
    for (uint8_t b = 0; b < 2; b++) {
        crc ^= data[b];
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// Lance une mesure (commande "no hold") et lit le résultat brut 16 bits
// après le délai de conversion requis. Retourne true si succès (ACK reçu,
// 3 octets reçus ET CRC valide).
static bool sht20_read_raw(uint8_t cmd, uint16_t delay_ms, uint16_t& raw_out) {
    Wire.beginTransmission(SHT20_ADDR);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) return false;

    delay(delay_ms);

    uint8_t received = Wire.requestFrom((int)SHT20_ADDR, 3);
    if (received < 3) return false; // pas assez d'octets reçus (CRC inclus)

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint8_t crc = Wire.read();

    if (sht20_crc8(msb, lsb) != crc) {
        Serial.println(F("[SHT20] CRC invalide - lecture ignorée (glitch I2C)"));
        return false;
    }

    raw_out = ((uint16_t)msb << 8) | (lsb & 0xFC); // 2 bits de statut à ignorer
    return true;
}

// Filtre anti-saut : rejette une valeur qui varierait de façon aberrante
// par rapport à la dernière lecture valide (cycle de 5s -> un vrai
// changement aussi rapide est physiquement impossible pour de l'air
// ambiant, donc c'est forcément un résidu de glitch non détecté par le CRC).
static const float MAX_DELTA_TEMP = 3.0f;  // °C entre 2 lectures consécutives
static const float MAX_DELTA_HUM  = 10.0f; // %RH entre 2 lectures consécutives

static bool sht20_get_temperature(float& out_c) {
    uint16_t raw;
    if (!sht20_read_raw(SHT20_CMD_TEMP_NOHOLD, 85, raw)) return false;
    float t = 175.72f * ((float)raw / 65536.0f) - 46.85f;
    if (!isnan(g_temperature) && fabsf(t - g_temperature) > MAX_DELTA_TEMP) {
        Serial.printf("[SHT20] Saut de température aberrant ignoré (%.1f -> %.1f)\n", g_temperature, t);
        return false;
    }
    out_c = t;
    return true;
}

static bool sht20_get_humidity(float& out_rh) {
    uint16_t raw;
    if (!sht20_read_raw(SHT20_CMD_HUM_NOHOLD, 29, raw)) return false;
    float h = 125.0f * ((float)raw / 65536.0f) - 6.0f;
    if (!isnan(g_humidite) && fabsf(h - g_humidite) > MAX_DELTA_HUM) {
        Serial.printf("[SHT20] Saut d'humidité aberrant ignoré (%.1f -> %.1f)\n", g_humidite, h);
        return false;
    }
    out_rh = h;
    return true;
}

// =====================================================================
// Mise à jour des labels du dashboard (pages.jsonl)
// Utilise dispatch_text_line(), le point d'entrée officiel openHASP pour
// injecter une commande "pXbY.attribut=valeur" depuis du code custom -
// exactement comme si la commande arrivait par MQTT, mais 100% locale.
// =====================================================================
static void update_dashboard_labels() {
    char buf[48];

    if (!isnan(g_temperature)) {
        snprintf(buf, sizeof(buf), "p1b3.text=%.1f °C", g_temperature);
        dispatch_text_line(buf, TAG_CUSTOM);
    }
    if (!isnan(g_humidite)) {
        snprintf(buf, sizeof(buf), "p1b2.text=%.0f %%", g_humidite);
        dispatch_text_line(buf, TAG_CUSTOM);
    }

    snprintf(buf, sizeof(buf), "p1b5.text=%.1f °C", g_consigne);
    dispatch_text_line(buf, TAG_CUSTOM);

#if SIMULATION_MODE
    snprintf(buf, sizeof(buf), "p1b9.text=%.0f %%", g_sim_vitesse);
    dispatch_text_line(buf, TAG_CUSTOM);

    snprintf(buf, sizeof(buf), "p1b11.text=%s", SIM_PROGRAMMES[g_sim_prog_index]);
    dispatch_text_line(buf, TAG_CUSTOM);
#endif
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

    // Consigne : rechargée depuis la NVS (survit à un reboot/reflash tant
    // que la partition NVS n'est pas effacée), sinon valeur par défaut 20°C.
    prefs.begin("daikin", false);
    g_consigne = prefs.getFloat("consigne", 20.0f);

    // Premier affichage du dashboard au boot (avant la première lecture
    // SHT20 à 5s, pour éviter un écran vide pendant les premières secondes).
    update_dashboard_labels();
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

    if (ok_t) g_temperature = t + TEMP_CALIBRATION_OFFSET;
    if (ok_h) g_humidite    = h;

    if (ok_t && ok_h) {
        Serial.printf("[SHT20] Température=%.1f°C  Humidité=%.1f%%\n", g_temperature, g_humidite);

        char pubbuf[16];
        snprintf(pubbuf, sizeof(pubbuf), "%.1f", g_temperature);
        dispatch_state_subtopic("temperature", pubbuf);

        snprintf(pubbuf, sizeof(pubbuf), "%.1f", g_humidite);
        dispatch_state_subtopic("humidite", pubbuf);
    } else {
        Serial.printf("[SHT20] Lecture échouée (temp=%s, hum=%s) - le capteur ne répond plus\n",
                      ok_t ? "OK" : "ECHEC", ok_h ? "OK" : "ECHEC");
        g_sht20_present = false; // on retentera un probe complet au prochain cycle
    }

#if SIMULATION_MODE
    // Vitesse ventilo simulée : oscille doucement entre 20% et 80% (onde
    // sinus lente sur millis(), rien à voir avec un vrai capteur/moteur).
    float phase = (float)(millis() % 60000) / 60000.0f * 2.0f * (float)PI;
    g_sim_vitesse = 50.0f + 30.0f * sinf(phase);

    // Programme de flux d'air simulé : change toutes les ~10s pour bien
    // voir le label bouger pendant le test.
    static uint32_t last_prog_change = 0;
    if (millis() - last_prog_change > 10000) {
        g_sim_prog_index = (g_sim_prog_index + 1) % SIM_PROGRAMMES_COUNT;
        last_prog_change = millis();
    }
#endif

    update_dashboard_labels();
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

// Reçoit les commandes envoyées vers le topic "custom" - déclenchées soit
// par MQTT (hasp/<plate>/command/custom), soit localement par un bouton
// pages.jsonl avec "action":{"up":"custom <commande>"} (mécanisme d'action
// locale documenté par openHASP, exécuté sans MQTT/broker).
//
// IMPORTANT - à vérifier au premier flash : le routage exact d'une action
// bouton locale vers ce hook n'est pas garanti à 100% par la documentation
// publique openHASP (elle documente le mécanisme "action" des boutons et
// séparément ce hook "custom_topic_payload" pour les messages MQTT, sans
// confirmer explicitement que les deux passent par le même chemin). Si les
// boutons +/- ou la réinitialisation WiFi ne réagissent pas au toucher,
// regarde les logs série au moment du clic : si rien ne s'affiche ici,
// c'est que l'action locale ne route pas vers custom_topic_payload sur ta
// version d'openHASP - dis-le moi avec le log et j'ajuste (probablement en
// passant par dispatch_text_line() direct depuis pages.jsonl, ou un autre
// hook de ton fork).
void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    (void)source;

    if (strcmp(topic, "custom") != 0) return;

    Serial.printf("[custom] Commande reçue : %s\n", payload);

    if (strcmp(payload, "consigne_plus") == 0) {
        g_consigne += CONSIGNE_STEP;
        if (g_consigne > CONSIGNE_MAX) g_consigne = CONSIGNE_MAX;
        prefs.putFloat("consigne", g_consigne);
        update_dashboard_labels();

    } else if (strcmp(payload, "consigne_moins") == 0) {
        g_consigne -= CONSIGNE_STEP;
        if (g_consigne < CONSIGNE_MIN) g_consigne = CONSIGNE_MIN;
        prefs.putFloat("consigne", g_consigne);
        update_dashboard_labels();

    } else if (strcmp(payload, "wifi_reset_confirm") == 0) {
        // Efface les identifiants WiFi stockés (NVS) et redémarre : openHASP
        // se retrouve sans réseau connu et rouvre son portail de config
        // (point d'accès + captive portal) au boot suivant - pas besoin de
        // reflasher pour changer de réseau WiFi.
        Serial.println(F("[custom] Réinitialisation WiFi demandée - redémarrage en mode config..."));
        delay(300);
        WiFi.disconnect(true, true);
        delay(300);
        ESP.restart();
    }
}
