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
#include <ArduinoJson.h>
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
// Page 1 : p1b2=humidité, p1b3=température, p1b5=consigne, p1b9=vitesse
// affichée, p1b11=programme/position affiché(e), p1b13=bouton "Regl." (page 3).
// Page 2 : confirmation réinitialisation WiFi.
// Page 3 : p3b3=sélecteur vitesse Auto/Manuel, p3b5=valeur vitesse manuelle,
// p3b6/p3b7=boutons -/+ vitesse manuelle, p3b9=sélecteur volet Auto/Fixe/Swing.
// À AJUSTER ICI si tu changes les id dans pages.jsonl.

// --- Consigne utilisateur (stockée en NVS pour survivre à un reboot) ---
static Preferences prefs;
static float g_consigne = 20.0f;
static const float CONSIGNE_MIN = 5.0f;
static const float CONSIGNE_MAX = 30.0f;
static const float CONSIGNE_STEP = 0.5f;

// --- Réglages vitesse ventilo / position volet (page 3 de pages.jsonl) ---
// "Auto" = calculé automatiquement (pour l'instant simulation - deviendra une
// vraie régulation une fois l'ESP32 secondaire/CAN câblé) ; "Manuel"/"Fixe"/
// "Swing" = valeur imposée par l'utilisateur via les sélecteurs de la page 3.
// Stockés en NVS comme la consigne, pour survivre à un reboot.
static bool    g_vitesse_manuelle_active = false; // false=Auto, true=Manuel
static float   g_vitesse_manuelle        = 50.0f; // % choisi en mode Manuel
static const float VITESSE_MANUELLE_STEP = 5.0f;
static const float VITESSE_MANUELLE_MIN  = 0.0f;
static const float VITESSE_MANUELLE_MAX  = 100.0f;

enum VoletMode : uint8_t { VOLET_AUTO = 0, VOLET_FIXE = 1, VOLET_SWING = 2 };
static uint8_t g_volet_mode = VOLET_AUTO;

// --- Valeurs simulées (mode simulation uniquement) ---
static float   g_sim_vitesse = 0.0f;
static uint8_t g_sim_prog_index = 0;
static const char* SIM_PROGRAMMES[] = { "Horizontal", "Oscillant", "Vertical" };
static const uint8_t SIM_PROGRAMMES_COUNT = 3;

// --- Configuration I2C (identique au bus tactile FT6336U déjà initialisé
// par le driver tactile officiel d'openHASP - on réutilise le même bus,
// pas de nouveau câblage nécessaire) ---
//
// v9 - VITESSE I2C RÉDUITE 400kHz -> 100kHz (ESPlogs 18/19/21) :
// Après avoir écarté l'alimentation comme cause unique (ESPlogs 18 : SHT20
// toujours ~21% de réussite même avec une alimentation externe distincte de
// l'Arduino Uno, alors que les crashs/brownouts eux avaient bien disparu),
// le message d'échec systématique "le capteur ne répond plus" (absence de
// réponse/timeout I2C, pas une donnée corrompue-mais-lue comme les CRC
// invalides vus avant) pointe vers un problème de signal sur le bus plutôt
// que d'alimentation pure. 400kHz est nettement plus exigeant en qualité de
// signal (temps de montée, pull-ups) que 100kHz pour un câblage/des
// composants qui ne sont peut-être pas garantis pour le "Fast Mode" I2C.
// Test à faible risque : ce Wire.begin() est déjà rappelé une seconde fois
// ici après l'init du driver tactile FT6336U (même bus physique partagé,
// cf commentaire ci-dessus) - le rebaisser à 100kHz ralentit donc aussi les
// transactions tactiles, mais 100kHz reste largement suffisant pour du
// tactile (bien plus lent que le taux de rafraîchissement de l'écran).
static const int SHT20_SDA_PIN   = 15;
static const int SHT20_SCL_PIN   = 6;
static const uint32_t I2C_FREQ   = 100000;
static const uint8_t  SHT20_ADDR = 0x40;

static const uint8_t SHT20_CMD_TEMP_NOHOLD = 0xF3;
static const uint8_t SHT20_CMD_HUM_NOHOLD  = 0xF5;

// Correction d'auto-échauffement : le capteur est sur le même PCB que le
// rétroéclairage/ESP32/WiFi, il lit donc plus chaud que l'air ambiant une
// fois le panneau chaud (observé sur ESPlogs 9 ET reconfirmé sur ESPlogs
// 10/11 : dérive continue de 33.3°C à 34.7°C+ en quelques minutes, et
// l'utilisateur confirme que la valeur affichée est trop élevée par rapport
// à la réalité). À calibrer : une fois le panneau stabilisé (15-20 min),
// comparer avec un thermomètre de référence et ajuster cette constante
// (ex. si le capteur affiche 31.5°C pour une vraie ambiante à 29.0°C,
// mettre -2.5f). Laissé à 0 tant que la mesure de référence n'est pas faite -
// calibration reportée volontairement à plus tard (confirmé par l'utilisateur).
static const float TEMP_CALIBRATION_OFFSET = 0.0f;

// --- État courant (mis à jour périodiquement) ---
// v13 : plus de tâche de fond séparée (voir plus bas) - tout tourne dans le
// même contexte que le reste d'openHASP (custom_loop()), donc plus besoin de
// spinlock/mutex pour protéger ces variables : un seul "fil d'exécution" y
// touche jamais concurremment.
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

// v13 : sht20_read_raw() (bloquante, avec delay() interne) a été scindée en
// deux étapes séparées - envoyer la commande, puis (après un délai géré par
// la machine à états de custom_loop(), SANS bloquer) lire le résultat - pour
// permettre une lecture SHT20 non bloquante. Voir le commentaire complet
// au-dessus de custom_loop() plus bas pour le contexte de ce changement.

// Étape 1 : envoie la commande de mesure ("no hold"). Retourne true si le
// capteur a accusé réception (ACK I2C sur l'adresse + la commande).
static bool sht20_send_cmd(uint8_t cmd) {
    Wire.beginTransmission(SHT20_ADDR);
    Wire.write(cmd);
    return (Wire.endTransmission() == 0);
}

// Étape 2 : à appeler après le délai de conversion requis (85ms température /
// 29ms humidité) - lit les 3 octets de résultat (MSB, LSB, CRC) et vérifie
// le CRC. Retourne true si succès (3 octets reçus ET CRC valide).
static bool sht20_read_result(uint16_t& raw_out) {
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
// par rapport à la dernière lecture valide (cycle de 5s) - sert surtout à
// filtrer un résidu de glitch non détecté par le CRC (une lecture isolée
// bizarre au milieu de lectures normales).
//
// v14 - GRÂCE APRÈS REJETS CONSÉCUTIFS (ESPlogs 27) : un vrai événement
// physique rapide (ex. souffle directement sur le capteur, confirmé par
// l'utilisateur) peut dépasser ce seuil sur PLUSIEURS lectures d'affilée,
// pas juste une seule. Avant v14, la référence restait figée sur l'ancienne
// valeur tant qu'aucune lecture ne repassait sous le seuil - jusqu'à 10-15s
// de blocage observés sur ESPlogs 27 pendant que la vraie valeur redescendait
// progressivement. Fix : un compteur de rejets consécutifs par grandeur
// (température/humidité) - après MAX_CONSECUTIVE_REJECTS rejets d'affilée,
// la lecture est acceptée telle quelle (probable vrai changement plutôt
// qu'un glitch isolé) et sert de nouvelle référence. Un glitch ponctuel
// (1 seule lecture aberrante) reste filtré comme avant.
static const float   MAX_DELTA_TEMP = 3.0f;  // °C entre 2 lectures consécutives
static const float   MAX_DELTA_HUM  = 10.0f; // %RH entre 2 lectures consécutives
static const uint8_t MAX_CONSECUTIVE_REJECTS = 2; // rejets tolérés avant d'accepter quand même
static uint8_t g_temp_reject_count = 0;
static uint8_t g_hum_reject_count  = 0;

// =====================================================================
// v13 - LECTURE SHT20 NON BLOQUANTE DANS custom_loop() (remplace la tâche
// FreeRTOS séparée des versions précédentes)
//
// HISTORIQUE (pour mémoire) : v4 avait déporté la lecture SHT20 (bloquante,
// ~114ms/cycle à cause des delay() de conversion) dans sa propre tâche
// FreeRTOS, pour ne jamais bloquer la boucle principale qui gère aussi
// l'écran/le tactile. RETOUR TERRAIN (ESPlogs 13) : une tentative ultérieure
// (v7) d'appeler update_dashboard_labels() depuis custom_loop() en plus de
// cette tâche a coïncidé avec des redémarrages en boucle (panics "Guru
// Meditation") - changement annulé (v8), cause exacte jamais formellement
// identifiée (pas de fichier .elf/.map pour décoder le backtrace).
//
// RÉVISION D'ARCHITECTURE (v13, suite à une suggestion externe pertinente
// de l'utilisateur) : la tâche FreeRTOS séparée pose un problème de fond que
// je n'avais pas assez pris en compte - elle touche `Wire` (le bus I2C)
// depuis un CONTEXTE D'EXÉCUTION DIFFÉRENT de celui du driver tactile
// FT6336U, qui partage EXACTEMENT le même bus physique (SDA=15/SCL=6) mais
// est piloté depuis la boucle principale d'openHASP. Sans aucune
// synchronisation entre les deux, un accès concurrent aux mêmes registres
// matériels I2C peut corrompre l'une ou l'autre transaction - hypothèse
// cohérente avec à la fois les échecs de lecture SHT20 chroniques ET les
// événements tactiles parasites observés (rebond du bouton "+", ESPlogs
// 22/23). Fix : suppression complète de `sht20_task()` et de son
// xTaskCreate() - la lecture SHT20 est maintenant une machine à états NON
// BLOQUANTE, avancée à chaque appel de custom_loop() (donc dans le MÊME
// contexte d'exécution que le reste d'openHASP, plus de contexte concurrent
// séparé touchant le bus I2C).
//
// ATTENTION - pour ne PAS reproduire le crash de v7 : cette machine à états
// mémorise seulement les valeurs (g_temperature/g_humidite) dans
// custom_loop() - elle n'appelle JAMAIS dispatch_text_line()/
// update_dashboard_labels() depuis custom_loop(). L'affichage continue à
// être poussé UNIQUEMENT par custom_every_5seconds() comme depuis v8, sur
// son tic habituel - seul l'accès I2C lui-même devient non bloquant/
// partagé-context-safe, le comportement d'affichage ne change pas.
// =====================================================================
enum Sht20State : uint8_t { SHT20_ST_IDLE = 0, SHT20_ST_TEMP_WAIT, SHT20_ST_HUM_WAIT };
static Sht20State g_sht20_state    = SHT20_ST_IDLE;
static uint32_t   g_sht20_next_ms  = 0;     // prochain instant où agir (millis())
static bool       g_sht20_ok_t     = false; // résultat température de ce cycle
static float      g_sht20_pending_t = NAN;  // température mesurée, en attente de l'humidité

// Termine un cycle en échec : force un nouveau probe complet au prochain
// tour, retente vite (1s) plutôt que d'attendre le cycle normal de 5s.
static void sht20_fail_cycle(bool ok_t, bool ok_h, uint32_t now) {
    Serial.printf("[SHT20] Lecture échouée (temp=%s, hum=%s) - le capteur ne répond plus\n",
                  ok_t ? "OK" : "ECHEC", ok_h ? "OK" : "ECHEC");
    g_sht20_present = false;
    g_sht20_state    = SHT20_ST_IDLE;
    g_sht20_next_ms  = now + 1000;
}

// Avance la machine à états SHT20 d'un pas si c'est l'heure. Appelée depuis
// custom_loop() à chaque itération - ne bloque jamais (pas de delay()).
static void sht20_state_machine_tick() {
    uint32_t now = millis();
    if ((int32_t)(now - g_sht20_next_ms) < 0) return; // pas encore l'heure

    switch (g_sht20_state) {
        case SHT20_ST_IDLE: {
            if (!g_sht20_present) {
                bool present = sht20_probe();
                g_sht20_present = present;
                if (!present) {
                    g_temperature = NAN;
                    g_humidite    = NAN;
                    g_sht20_next_ms = now + 5000; // pas de capteur : on retente lentement
                    return;
                }
                Serial.println(F("[SHT20] Capteur détecté (nouvelle tentative)"));
            }

            if (!sht20_send_cmd(SHT20_CMD_TEMP_NOHOLD)) {
                sht20_fail_cycle(false, false, now);
                return;
            }
            g_sht20_state   = SHT20_ST_TEMP_WAIT;
            g_sht20_next_ms = now + 85; // délai de conversion température
            break;
        }

        case SHT20_ST_TEMP_WAIT: {
            uint16_t raw;
            bool ok = sht20_read_result(raw);
            if (ok) {
                float t = 175.72f * ((float)raw / 65536.0f) - 46.85f;
                bool jump = !isnan(g_temperature) && fabsf(t - g_temperature) > MAX_DELTA_TEMP;
                if (jump && g_temp_reject_count < MAX_CONSECUTIVE_REJECTS) {
                    g_temp_reject_count++;
                    Serial.printf("[SHT20] Saut de température aberrant ignoré (%.1f -> %.1f, rejet %u/%u)\n",
                                  g_temperature, t, g_temp_reject_count, MAX_CONSECUTIVE_REJECTS);
                    ok = false;
                } else {
                    if (jump) {
                        Serial.printf("[SHT20] Saut de température accepté après %u rejets consécutifs (%.1f -> %.1f) - probable variation réelle\n",
                                      g_temp_reject_count, g_temperature, t);
                    }
                    g_temp_reject_count = 0;
                    g_sht20_pending_t = t;
                }
            }
            g_sht20_ok_t = ok;

            // On tente quand même l'humidité même si la température a échoué
            // (comportement identique aux versions précédentes - un échec
            // isolé sur un seul des deux ne veut pas forcément dire que le
            // capteur ne répond plus du tout).
            if (!sht20_send_cmd(SHT20_CMD_HUM_NOHOLD)) {
                sht20_fail_cycle(g_sht20_ok_t, false, now);
                return;
            }
            g_sht20_state   = SHT20_ST_HUM_WAIT;
            g_sht20_next_ms = now + 29; // délai de conversion humidité
            break;
        }

        case SHT20_ST_HUM_WAIT: {
            uint16_t raw;
            bool ok_h = sht20_read_result(raw);
            float h = NAN;
            if (ok_h) {
                h = 125.0f * ((float)raw / 65536.0f) - 6.0f;
                bool jump = !isnan(g_humidite) && fabsf(h - g_humidite) > MAX_DELTA_HUM;
                if (jump && g_hum_reject_count < MAX_CONSECUTIVE_REJECTS) {
                    g_hum_reject_count++;
                    Serial.printf("[SHT20] Saut d'humidité aberrant ignoré (%.1f -> %.1f, rejet %u/%u)\n",
                                  g_humidite, h, g_hum_reject_count, MAX_CONSECUTIVE_REJECTS);
                    ok_h = false;
                } else if (jump) {
                    Serial.printf("[SHT20] Saut d'humidité accepté après %u rejets consécutifs (%.1f -> %.1f) - probable variation réelle\n",
                                  g_hum_reject_count, g_humidite, h);
                    g_hum_reject_count = 0;
                } else {
                    g_hum_reject_count = 0;
                }
            }

            if (g_sht20_ok_t && ok_h) {
                g_temperature = g_sht20_pending_t + TEMP_CALIBRATION_OFFSET;
                g_humidite    = h;
                Serial.printf("[SHT20] Température=%.1f°C  Humidité=%.1f%%\n", g_temperature, g_humidite);
                g_sht20_state   = SHT20_ST_IDLE;
                g_sht20_next_ms = now + 5000; // lecture réussie : cadence normale
            } else {
                sht20_fail_cycle(g_sht20_ok_t, ok_h, now);
            }
            break;
        }
    }
}

// =====================================================================
// Mise à jour des labels du dashboard (pages.jsonl)
// Utilise dispatch_text_line(), le point d'entrée officiel openHASP pour
// injecter une commande "pXbY.attribut=valeur" depuis du code custom -
// exactement comme si la commande arrivait par MQTT, mais 100% locale.
// =====================================================================
static void update_dashboard_labels() {
    char buf[48];

    // v13 : plus de tâche de fond séparée, donc plus besoin de spinlock -
    // g_temperature/g_humidite sont mises à jour uniquement depuis
    // custom_loop() (même contexte que cette fonction), lecture directe OK.
    float temp_snapshot = g_temperature;
    float hum_snapshot  = g_humidite;

    if (!isnan(temp_snapshot)) {
        snprintf(buf, sizeof(buf), "p1b3.text=%.1f °C", temp_snapshot);
        dispatch_text_line(buf, TAG_CUSTOM);
    }
    if (!isnan(hum_snapshot)) {
        snprintf(buf, sizeof(buf), "p1b2.text=%.0f %%", hum_snapshot);
        dispatch_text_line(buf, TAG_CUSTOM);
    }

    snprintf(buf, sizeof(buf), "p1b5.text=%.1f °C", g_consigne);
    dispatch_text_line(buf, TAG_CUSTOM);

#if SIMULATION_MODE
    // Vitesse affichée : la valeur manuelle si l'utilisateur a choisi "Manuel"
    // sur la page 3, sinon la simulation "Auto" habituelle (sinusoïde).
    snprintf(buf, sizeof(buf), "p1b9.text=%.0f %%",
             g_vitesse_manuelle_active ? g_vitesse_manuelle : g_sim_vitesse);
    dispatch_text_line(buf, TAG_CUSTOM);

    // Position du volet affichée : Fixe/Swing si choisi explicitement, sinon
    // le cycle de simulation habituel en mode "Auto".
    const char* volet_text;
    switch (g_volet_mode) {
        case VOLET_FIXE:  volet_text = "Fixe";      break;
        case VOLET_SWING: volet_text = "Oscillant"; break;
        default:          volet_text = SIM_PROGRAMMES[g_sim_prog_index]; break; // VOLET_AUTO
    }
    snprintf(buf, sizeof(buf), "p1b11.text=%s", volet_text);
    dispatch_text_line(buf, TAG_CUSTOM);

    // Réglages page 3 : valeur manuelle toujours affichée là-bas, qu'elle soit
    // active ou non (sert de pré-réglage prêt à activer).
    snprintf(buf, sizeof(buf), "p3b5.text=%.0f %%", g_vitesse_manuelle);
    dispatch_text_line(buf, TAG_CUSTOM);
#endif
}

// =====================================================================
// Points d'accroche appelés automatiquement par openHASP
// =====================================================================

void custom_setup() {
    // v13 : le bus I2C (SDA=15/SCL=6) est déjà initialisé par le driver
    // tactile FT6336U d'openHASP avant l'appel à custom_setup(). On ne
    // rappelle plus Wire.begin() ici (suggestion reçue et retenue : éviter
    // de ré-initialiser un bus déjà configuré) ; si besoin de forcer la
    // fréquence I2C, on utilise setClock() qui n'a pas cet effet de bord.
    Wire.setClock(I2C_FREQ);

    g_sht20_present = sht20_probe();
    if (g_sht20_present) {
        Serial.println(F("[SHT20] Capteur détecté à l'adresse 0x40 (bus I2C tactile partagé)"));
    } else {
        Serial.println(F("[SHT20] AUCUN capteur détecté à l'adresse 0x40 - vérifier que la puce est bien montée sur ce panneau"));
    }

    // v13 : plus de tâche FreeRTOS séparée pour la lecture SHT20 - la
    // lecture I2C est maintenant une machine à états non bloquante avancée
    // depuis custom_loop() (voir sht20_state_machine_tick() plus haut),
    // dans le MÊME contexte que le driver tactile, pour éliminer tout accès
    // concurrent non synchronisé au bus I2C partagé.

    // Consigne : rechargée depuis la NVS (survit à un reboot/reflash tant
    // que la partition NVS n'est pas effacée), sinon valeur par défaut 20°C.
    prefs.begin("daikin", false);
    g_consigne                = prefs.getFloat("consigne", 20.0f);
    g_vitesse_manuelle_active = prefs.getBool("vit_man_on", false);
    g_vitesse_manuelle        = prefs.getFloat("vit_man_val", 50.0f);
    g_volet_mode              = prefs.getUChar("volet_mode", VOLET_AUTO);

    // Premier affichage du dashboard au boot (avant la première lecture
    // SHT20 à 5s, pour éviter un écran vide pendant les premières secondes).
    update_dashboard_labels();
}

void custom_loop() {
    // v13 : fait avancer la machine à états de lecture SHT20 (non bloquante,
    // basée sur millis()) à chaque itération de la boucle principale
    // d'openHASP - donc dans le MÊME contexte que le driver tactile, ce qui
    // élimine l'accès concurrent non synchronisé au bus I2C partagé qui
    // existait avec l'ancienne tâche FreeRTOS séparée sht20_task().
    //
    // RÈGLE CRITIQUE (héritée de la v7, NE JAMAIS ENFREINDRE) : cette
    // fonction ne doit JAMAIS appeler update_dashboard_labels() ni
    // dispatch_text_line(). Un essai précédent qui le faisait a coïncidé
    // avec des redémarrages en boucle (ESPlogs 13). sht20_state_machine_tick()
    // se contente de mettre à jour des variables internes (g_temperature,
    // g_humidite, ...) - l'affichage reste exclusivement poussé par
    // custom_every_5seconds(), comme depuis la v8.
    sht20_state_machine_tick();
}

void custom_every_second() {
    // pas utilisé pour l'instant
}

// Exécutée dans la boucle principale d'openHASP (celle qui gère aussi
// l'écran/le tactile) - ne doit JAMAIS bloquer. Ne fait plus aucun accès
// I2C direct : elle se contente de publier l'état déjà lu par la machine à
// états SHT20 (sht20_state_machine_tick(), avancée dans custom_loop()) et
// de faire avancer les valeurs simulées.
void custom_every_5seconds() {
    // v13 : plus de spinlock nécessaire, même contexte que custom_loop().
    float temp_snapshot = g_temperature;
    float hum_snapshot  = g_humidite;

    if (!isnan(temp_snapshot) && !isnan(hum_snapshot)) {
        char pubbuf[16];
        snprintf(pubbuf, sizeof(pubbuf), "%.1f", temp_snapshot);
        dispatch_state_subtopic("temperature", pubbuf);

        snprintf(pubbuf, sizeof(pubbuf), "%.1f", hum_snapshot);
        dispatch_state_subtopic("humidite", pubbuf);
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

// Point d'accroche appelé PAR le noyau openHASP à CHAQUE publication d'état
// interne (dispatch_state_subtopic()) - donc pour tous les objets qui ne
// passent PAS par le mécanisme "action" des simples boutons "btn".
//
// CONFIRMÉ par lecture du code source openHASP (hasp_event.cpp / hasp_object.cpp/
// hasp_dispatch.cpp) : les objets interactifs "btnmatrix", "slider", "roller",
// "dropdown" et "switch" n'exécutent JAMAIS leur "action" JSON (ce mécanisme,
// via script_event_handler(), n'est câblé QUE sur generic_event_handler,
// utilisé par les objets "btn" simples - exactement ce qu'on utilise déjà
// pour les boutons +/- et le reset WiFi). Ces objets-là publient uniquement
// leur nouvel état ici, sous forme topic="p<page>b<id>" et
// payload=JSON {"event":"changed","val":<index>,"text":"<label sélectionné>"}.
// C'est donc ICI, et pas dans custom_topic_payload(), qu'on récupère les
// sélecteurs "Auto/Manuel" (vitesse) et "Auto/Fixe/Swing" (volet) de la
// page 3 (objets "btnmatrix" avec toggle+one_check dans pages.jsonl).
void custom_state_subtopic(const char* subtopic, const char* payload) {
    if (strcmp(subtopic, "p3b3") == 0) {
        // Sélecteur vitesse ventilo : options ["Auto","Manuel"] -> val 0 ou 1
        StaticJsonDocument<128> doc;
        if (deserializeJson(doc, payload) != DeserializationError::Ok) return;
        int val = doc["val"] | -1;
        if (val < 0) return;

        g_vitesse_manuelle_active = (val == 1);
        prefs.putBool("vit_man_on", g_vitesse_manuelle_active);
        Serial.printf("[custom] Mode vitesse ventilo -> %s\n", g_vitesse_manuelle_active ? "Manuel" : "Auto");
        update_dashboard_labels();

    } else if (strcmp(subtopic, "p3b9") == 0) {
        // Sélecteur position volet : options ["Auto","Fixe","Swing"] -> val 0/1/2
        StaticJsonDocument<128> doc;
        if (deserializeJson(doc, payload) != DeserializationError::Ok) return;
        int val = doc["val"] | -1;
        if (val < 0 || val > 2) return;

        g_volet_mode = (uint8_t)val;
        prefs.putUChar("volet_mode", g_volet_mode);
        Serial.printf("[custom] Position volet -> %u\n", g_volet_mode);
        update_dashboard_labels();
    }
}

// Ajoute nos valeurs au message de capteurs périodique d'openHASP
void custom_get_sensors(JsonDocument& doc) {
    // v13 : plus de spinlock nécessaire (lecture faite dans le même
    // contexte que l'écriture, cf custom_loop()/sht20_state_machine_tick()).
    float temp_snapshot    = g_temperature;
    float hum_snapshot     = g_humidite;
    bool  present_snapshot = g_sht20_present;

    JsonObject sensor = doc.createNestedObject(F("sht20"));
    sensor[F("temperature")] = temp_snapshot;
    sensor[F("humidite")]    = hum_snapshot;
    sensor[F("present")]     = present_snapshot;
}

// Reçoit les commandes routées vers "custom/<sous-topic>" - déclenchées soit
// par MQTT (hasp/<plate>/command/custom/<sous-topic>), soit localement par un
// bouton pages.jsonl avec "action":{"up":"custom/<sous-topic>=<valeur>"}.
//
// CONFIRMÉ le [test réel du 24/08] par lecture du code source officiel
// openHASP (dispatch_topic_payload() dans src/hasp/hasp_dispatch.cpp) : le
// routeur ne reconnaît PAS le mot "custom" comme commande - il teste si le
// topic COMMENCE PAR le préfixe littéral "custom/" (avec le slash), retire
// ces 7 premiers caractères, puis appelle custom_topic_payload() avec le
// RESTE du topic (déjà débarrassé du préfixe) comme paramètre `topic`. C'est
// donc bien `topic` qu'il faut tester ici, pas `payload` - et pages.jsonl doit
// écrire "custom/consigne_plus=1" (pas "custom consigne_plus"). Les logs
// ESPlogs 10/11 montraient exactement l'erreur inverse : "Command 'custom'
// not found => consigne_plus", preuve que le bouton atteignait bien le
// dispatcher mais avec la mauvaise syntaxe de topic.
//
// v10 - ANTI-REBOND (ESPlogs 22) : l'utilisateur a signalé qu'un seul appui
// sur "+" fait parfois monter la consigne de 2 crans au lieu d'un. Analyse du
// log : sur la séquence de "consigne_plus" reçus, deux paires consécutives
// sont espacées d'EXACTEMENT 264ms (76.551s->76.815s, puis 91.159s->91.423s,
// à la milliseconde près) alors que tous les autres écarts entre appuis
// réels vont de ~0.9s à plusieurs secondes. Un écart identique au ms près à
// deux moments différents ne peut pas être un double-appui humain (jamais
// aussi régulier) - c'est la signature d'un double événement généré par le
// driver tactile (rebond) en amont de ce fichier, pas un bug dans la logique
// consigne_plus/moins elle-même (chaque commande reçue est bien traitée
// correctement une seule fois). Fix : anti-rebond logiciel, ignore une
// commande identique à la précédente si elle arrive à moins de 300ms
// d'écart.
//
// v11 - ANTI-REBOND AMÉLIORÉ (ESPlogs 23) : v10 fonctionne bien sur les cas
// simples (le log confirme plusieurs "ignorée (rebond, 205-272ms...)" -
// preuve qu'elle attrape bien le rebond immédiat), MAIS l'utilisateur a
// signalé qu'en tapant 4-5 fois rapidement de suite, la consigne continue à
// grimper toute seule au-delà de ce qu'il a appuyé (jusqu'à 30°C en fin de
// test). Cause du trou dans v10 : le chrono n'était réarmé QUE sur une
// commande acceptée, jamais sur une commande ignorée. Or lors d'une rafale
// d'appuis rapprochés, un rebond "tardif" (~350-450ms après l'appui accepté,
// donc hors de la fenêtre de 300ms) se retrouvait comparé à un chrono resté
// figé sur l'appui accepté précédent, et passait pour un nouvel appui
// légitime. Fix : fenêtre glissante - le chrono est maintenant réarmé à
// CHAQUE occurrence de la même commande (acceptée OU ignorée), pas
// seulement les acceptées. Un rebond tardif qui arrive quand même dans les
// 400ms du dernier événement (accepté ou non) est donc lui aussi filtré, et
// toute la rafale de rebonds d'un même appui physique s'écrase en un seul
// appui compté - alors qu'un vrai appui suivant, séparé de plus de 400ms du
// dernier événement (rebond compris), redémarre une fenêtre fraîche et est
// bien accepté. Seuil remonté à 400ms (marge au-dessus des rebonds tardifs
// observés) - les appuis répétés volontaires les plus rapides observés dans
// les logs restent nettement au-dessus (~500ms+).
//
// v12 - SEUIL RENFORCÉ (retour terrain post-v11) : l'utilisateur précise que
// le phénomène n'est pas juste "1 ou 2 crans en trop" mais une consigne qui
// continue à grimper de 0.5 en 0.5 sur plusieurs secondes après UN SEUL
// appui sur "+" (le même test sur "-" ne reproduit rien). Ça ressemble
// moins à un simple rebond mécanique qu'à un vrai train d'événements répétés
// (éventuellement un "appui resté collé" côté driver tactile) qui peut durer
// plus longtemps que les 400ms de v11. Faute de log capturant précisément
// cet épisode (durée exacte, écart entre les crans en trop), je ne peux pas
// encore affiner le seuil avec la même précision qu'en v10/v11 - je monte
// donc le seuil à 2000ms par prudence (la fenêtre glissante veut dire que
// tant que les impulsions parasites arrivent à moins de 2s les unes des
// autres, tout le train s'écrase en 1 seul appui compté, quelle que soit sa
// durée totale). Contrepartie assumée : un vrai double-appui volontaire très
// rapide (moins de 2s d'écart) sera aussi ignoré - jugé acceptable pour une
// consigne de température (mieux vaut réappuyer une deuxième fois après 2s
// que risquer une dérive incontrôlée). À affiner avec un ESPlogs capturant
// l'épisode complet (un appui sur "+", laissé dériver) si le phénomène
// persiste malgré ce seuil.
//
// v14 (essai, ABANDONNÉ) - baissé un temps à 700ms suite à un retour
// utilisateur sur ESPlogs 27, mais réflexion faite : 2000ms protège très
// bien (aucune dérive sur ESPlogs 27) et le risque d'un pas en trop
// occasionnel à 700ms (rafales jusqu'à ~1.5s observées sur ce même log)
// n'était pas justifié tant que la propreté du signal tactile après v13
// n'est pas confirmée sur plus de tests.
//
// v15 - RETOUR À 2000ms, réduction PROGRESSIVE prévue : décision utilisateur
// de garder la marge de sécurité maximale pour l'instant, et de tester des
// seuils plus bas (500-700ms, voire moins) par étapes UNE FOIS le tactile
// confirmé stable sur plusieurs tests consécutifs post-v13 - plutôt que de
// baisser le seuil en même temps que d'autres changements non encore
// éprouvés sur la durée.
static const uint32_t COMMAND_DEBOUNCE_MS = 2000;
static char     g_last_topic[32] = "";
static uint32_t g_last_topic_time = 0;

void custom_topic_payload(const char* topic, const char* payload, uint8_t source) {
    (void)source;
    (void)payload;

    uint32_t now = millis();
    bool is_bounce = (strcmp(topic, g_last_topic) == 0) && ((now - g_last_topic_time) < COMMAND_DEBOUNCE_MS);
    uint32_t gap = now - g_last_topic_time; // pour le log, avant de réarmer le chrono

    // Fenêtre glissante : on réarme le chrono à CHAQUE occurrence (acceptée
    // ou rebond), pas seulement sur acceptation - voir commentaire v11.
    strncpy(g_last_topic, topic, sizeof(g_last_topic) - 1);
    g_last_topic[sizeof(g_last_topic) - 1] = '\0';
    g_last_topic_time = now;

    if (is_bounce) {
        Serial.printf("[custom] Commande '%s' ignorée (rebond, %lums après la précédente)\n",
                      topic, (unsigned long)gap);
        return;
    }

    Serial.printf("[custom] Commande reçue : %s\n", topic);

    if (strcmp(topic, "consigne_plus") == 0) {
        g_consigne += CONSIGNE_STEP;
        if (g_consigne > CONSIGNE_MAX) g_consigne = CONSIGNE_MAX;
        prefs.putFloat("consigne", g_consigne);
        update_dashboard_labels();

    } else if (strcmp(topic, "consigne_moins") == 0) {
        g_consigne -= CONSIGNE_STEP;
        if (g_consigne < CONSIGNE_MIN) g_consigne = CONSIGNE_MIN;
        prefs.putFloat("consigne", g_consigne);
        update_dashboard_labels();

    } else if (strcmp(topic, "vitesse_manuelle_plus") == 0) {
        g_vitesse_manuelle += VITESSE_MANUELLE_STEP;
        if (g_vitesse_manuelle > VITESSE_MANUELLE_MAX) g_vitesse_manuelle = VITESSE_MANUELLE_MAX;
        prefs.putFloat("vit_man_val", g_vitesse_manuelle);
        update_dashboard_labels();

    } else if (strcmp(topic, "vitesse_manuelle_moins") == 0) {
        g_vitesse_manuelle -= VITESSE_MANUELLE_STEP;
        if (g_vitesse_manuelle < VITESSE_MANUELLE_MIN) g_vitesse_manuelle = VITESSE_MANUELLE_MIN;
        prefs.putFloat("vit_man_val", g_vitesse_manuelle);
        update_dashboard_labels();

    } else if (strcmp(topic, "wifi_reset_confirm") == 0) {
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
