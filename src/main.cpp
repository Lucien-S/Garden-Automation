// ============================================================
// GARDEN CONTROLLER — ESP32
// Serial robustness:
//   • Wizard (setup)  → Serial.setTimeout(30000) + readStringUntil('\n')
//     Le blocage est VOULU ici, on attend l'humain.
//   • Loop (runtime)  → char-by-char buffer non-bloquant
//     Commandes ex: "WATER:1", "STOP", "STATUS"
//
// Supprimé : setjmp/longjmp (UB avec objets C++), goto (UB cross-declaration)
// Remplacé par : do/while(restart) + boucles while + fonctions helper
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <cstdio>
#include <Preferences.h>
#include "RTClib.h"

// ============================================================
// CONSTANTS
// ============================================================
namespace cfg {
    constexpr char     AP_SSID[]          = "Colloc";
    constexpr char     AP_PASSWORD[]      = "essai1234";
    constexpr size_t   AP_PASSWORD_LEN    = sizeof(AP_PASSWORD) - 1;
    const     IPAddress AP_LOCAL_IP       = IPAddress(192,168,4,1);
    const     IPAddress AP_GATEWAY        = IPAddress(192,168,4,1);
    const     IPAddress AP_SUBNET         = IPAddress(255,255,255,0);

    constexpr char     MQTT_HOST[]        = "192.168.4.2";
    constexpr uint16_t MQTT_PORT          = 1883;
    constexpr char     MQTT_CLIENT_ID[]   = "esp32-garden";
    constexpr char     MQTT_CMD_TOPIC[]   = "garden/valve/command";
    constexpr unsigned long PUBLISH_MS    = 300000UL;  // 5 min

    constexpr byte     SENSOR1_ADDR       = 0x01;
    constexpr byte     SENSOR2_ADDR       = 0x02;

    constexpr uint8_t  OLED_SDA           = 32;
    constexpr uint8_t  OLED_SCL           = 33;
    constexpr int8_t   OLED_RESET         = -1;
    constexpr uint8_t  SCREEN_W           = 128;
    constexpr uint8_t  SCREEN_H           = 32;

    constexpr uint8_t  PUMP_PIN           = 23;
    constexpr uint8_t  EV1_PIN            = 19;
    constexpr uint8_t  EV2_PIN            = 22;
    constexpr uint8_t  EV3_PIN            = 3;

    constexpr uint8_t  RTC_SDA            = 25;
    constexpr uint8_t  RTC_SCL            = 26;

    constexpr float    HUMIDITY_THRESHOLD = 30.0f;
    constexpr unsigned long PRIME_DURATION_MS = 15000UL;  // amorçage pompe avant ouverture EV
    constexpr unsigned long WATER_DUR_MS      = 30000UL;
    constexpr unsigned long COOLDOWN_MS       = 120000UL;
    constexpr unsigned long SENSOR_READ_MS = 30000UL;  // lecture capteurs
    constexpr unsigned long SENSOR_LOG_MS  = 60000UL;  // affichage série
}

// ============================================================
// STRUCTS
// ============================================================
struct SensorReading {
    float humidity    = 0.0f;
    float temperature = 0.0f;
    bool  ok          = false;
};

struct CalibData {
    float s1Dry = 0.0f, s1Wet = 100.0f;
    float s2Dry = 0.0f, s2Wet = 100.0f;
};

struct Config {
    uint8_t evMaskSensor1 = 0b001;  // bitmask: bit0=EV1 bit1=EV2 bit2=EV3
    uint8_t evMaskSensor2 = 0b100;
    uint8_t wateringDays  = 10;
};

struct WateringState {
    bool          pumpActive               = false;
    bool          isPriming               = false;  // true pendant les 15 s d'amorçage
    uint8_t       activeValveMask          = 0;
    uint8_t       activeZone               = 0;    // 1 = capteur1, 2 = capteur2
    unsigned long cycleStartTime           = 0;
    unsigned long primingStartTime         = 0;
    unsigned long maxDuration              = 0;
    unsigned long lastCycleTime[3]         = {0, 0, 0};
};

// ============================================================
// GLOBALS
// ============================================================
Preferences      prefs;
HardwareSerial   rs485Serial(2);
Adafruit_SSD1306 display(cfg::SCREEN_W, cfg::SCREEN_H, &Wire, cfg::OLED_RESET);
WiFiClient       wifiClient;
PubSubClient     mqttClient(wifiClient);
TwoWire          rtcWire(1);   // I2C bus 1 pour la RTC (SDA=25, SCL=26)
RTC_DS3231       rtc;
bool             rtcAvailable = false;

byte          response[20];
unsigned long lastPublish      = 0;
unsigned long lastSensorRead   = 0;
unsigned long lastSensorLog    = 0;
unsigned long lastDailyCheck   = 0;
unsigned long lastAlertTime    = 0;
unsigned long lastMqttAttempt  = 0;
bool   pendingPostWateringPublish = false;
CalibData     calibData;
Config        config;
WateringState wateringState;

// Buffer pour commandes série non-bloquantes dans loop()
static char   serialBuf[64];
static uint8_t serialBufIdx = 0;

// ============================================================
// NVS
// ============================================================
constexpr char NVS_NS[]         = "garden";
constexpr char NVS_CALIBRATED[] = "calibrated";
constexpr char NVS_S1_DRY[]     = "s1_dry";
constexpr char NVS_S1_WET[]     = "s1_wet";
constexpr char NVS_S2_DRY[]     = "s2_dry";
constexpr char NVS_S2_WET[]     = "s2_wet";
constexpr char NVS_EV_MASK1[]   = "ev_mask1";
constexpr char NVS_EV_MASK2[]   = "ev_mask2";
constexpr char NVS_WATER_DAYS[] = "water_days";
constexpr char NVS_DAILY_USED_Z1[] = "used_z1";
constexpr char NVS_DAILY_USED_Z2[] = "used_z2";
constexpr char NVS_LAST_DAY[]      = "last_day";

void loadNVS() {
    prefs.begin(NVS_NS, true);
    calibData.s1Dry     = prefs.getFloat(NVS_S1_DRY, 0.0f);
    calibData.s1Wet     = prefs.getFloat(NVS_S1_WET, 100.0f);
    calibData.s2Dry     = prefs.getFloat(NVS_S2_DRY, 0.0f);
    calibData.s2Wet     = prefs.getFloat(NVS_S2_WET, 100.0f);
    config.evMaskSensor1 = prefs.getUChar(NVS_EV_MASK1, 0b001);
    config.evMaskSensor2 = prefs.getUChar(NVS_EV_MASK2, 0b100);
    config.wateringDays = prefs.getUChar(NVS_WATER_DAYS, 10);
    prefs.end();
}

void saveNVS() {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(NVS_S1_DRY,    calibData.s1Dry);
    prefs.putFloat(NVS_S1_WET,    calibData.s1Wet);
    prefs.putFloat(NVS_S2_DRY,    calibData.s2Dry);
    prefs.putFloat(NVS_S2_WET,    calibData.s2Wet);
    prefs.putUChar(NVS_EV_MASK1,   config.evMaskSensor1);
    prefs.putUChar(NVS_EV_MASK2,   config.evMaskSensor2);
    prefs.putUChar(NVS_WATER_DAYS, config.wateringDays);
    prefs.putBool(NVS_CALIBRATED,  true);
    prefs.end();
}

bool isCalibrated() {
    prefs.begin(NVS_NS, true);
    bool v = prefs.getBool(NVS_CALIBRATED, false);
    prefs.end();
    return v;
}

float getDailyUsedL(uint8_t zone) {
    prefs.begin(NVS_NS, true);
    const char* key = (zone == 1) ? NVS_DAILY_USED_Z1 : NVS_DAILY_USED_Z2;
    float v = prefs.isKey(key) ? prefs.getFloat(key, 0.0f) : 0.0f;
    prefs.end();
    return v;
}

void setDailyUsedL(uint8_t zone, float v) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(zone == 1 ? NVS_DAILY_USED_Z1 : NVS_DAILY_USED_Z2, v);
    prefs.end();
}

void resetDailyUsedIfNewDay() {
    if (!rtcAvailable) return;  // sans RTC on ne connait pas le jour
    uint8_t today = rtc.now().day();
    prefs.begin(NVS_NS, false);
    uint8_t last = prefs.getUChar(NVS_LAST_DAY, 0);
    if (last != today) {
        prefs.putFloat(NVS_DAILY_USED_Z1, 0.0f);
        prefs.putFloat(NVS_DAILY_USED_Z2, 0.0f);
        prefs.putUChar(NVS_LAST_DAY, today);
        Serial.println("Nouveau jour: budgets zones remis a zero.");
    }
    prefs.end();
}

// Forward declarations (définies plus bas)
static String  evMaskToStr(uint8_t mask);
static float   computeZoneMax(uint8_t zone);
void publishAlert(const char* msg);
void publishPostWateringStatus();

// ============================================================
// SERIAL — WIZARD (bloquant, utilisé dans setup() uniquement)
// ============================================================
// Principe : Serial.setTimeout(30000) défini une fois dans setup().
// readStringUntil('\n') bloque jusqu'au '\n' ou timeout 30s.
// trim() gère \r\n, \n, ou pas de terminateur selon réglage IDE.

// Vide le buffer entrant
void clearSerial() {
    while (Serial.available()) Serial.read();
    delay(10); // laisse les derniers octets arriver
    while (Serial.available()) Serial.read();
}

// Lit une ligne. Retourne "" si timeout ou ligne vide.
// À utiliser UNIQUEMENT depuis setup()/wizard.
static String wizReadLine() {
    String s = "";
    unsigned long start = millis();
    while (millis() - start < 30000) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                s.trim();
                if (s.length() > 0) {
                    Serial.println();  // newline après la saisie
                    delay(5);
                    while (Serial.available()) {
                        char next = (char)Serial.peek();
                        if (next == '\r' || next == '\n') Serial.read();
                        else break;
                    }
                    return s;
                }
                // s still empty: lone \r or \n before content, keep reading
            } else {
                s += c;
                Serial.write(c);  // echo du caractère → l'utilisateur voit ce qu'il tape
            }
        }
        yield();    // nourrit le watchdog FreeRTOS, évite le reboot silencieux
        delay(10);
    }
    s.trim();
    return s;
}

// Attend un caractère unique. Ignore les CR/LF isolés.
// Retourne '\0' après 30 s de silence.
static char wizReadChar() {
    unsigned long start = millis();
    while (millis() - start < 30000) {
        if (Serial.available()) {
            char c = (char)Serial.read();
            if (c != '\r' && c != '\n') {
                // Drain any trailing CR/LF
                delay(10);
                while (Serial.available()) {
                    char nx = (char)Serial.peek();
                    if (nx == '\r' || nx == '\n') Serial.read();
                    else break;
                }
                return (char)tolower(c);
            }
            // Lone CR or LF — ignore, keep waiting
        }
        yield();
        delay(10);
    }
    return '\0';
}

// Demande o/n. Retourne true si 'o', false si 'n'.
// Répète jusqu'à réponse valide ou maxTries dépassé (renvoie defaultVal).
static bool wizAskON(const char* question, bool defaultVal = false) {
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        Serial.print(question);
        Serial.print(" (o/n) : ");
        char c = wizReadChar();
        if (c == '\0') { Serial.println("  (rien recu, reessayez)"); continue; }
        if (c == 'o') { Serial.println("o"); return true;  }
        if (c == 'n') { Serial.println("n"); return false; }
        Serial.printf("  -> '%c' non reconnu\n", c);
    }
    Serial.printf("  -> Defaut : %s\n", defaultVal ? "oui" : "non");
    return defaultVal;
}

// Lit un entier. Répète si invalide.
static int wizReadInt(const char* prompt, int minVal, int maxVal) {
    while (true) {
        Serial.print(prompt);
        String s = wizReadLine();
        if (s.length() == 0) { Serial.println("  (rien recu, reessayez)"); continue; }
        // toInt() retourne 0 si non-numérique → on vérifie que le 1er char est un chiffre
        if (!isDigit(s[0]) && !(s[0] == '-')) { Serial.println("  -> Pas un nombre."); continue; }
        int v = s.toInt();
        if (v < minVal || v > maxVal) {
            Serial.printf("  -> Hors plage [%d-%d]\n", minVal, maxVal);
            continue;
        }
        Serial.printf("  -> %d\n", v);
        return v;
    }
}

// Lit un masque d'EV. Accepte "1", "2", "3", "1/2", "1/3", "2/3", "1/2/3".
// Retourne un bitmask (bit0=EV1, bit1=EV2, bit2=EV3).
static uint8_t wizReadEVMask(const char* prompt) {
    while (true) {
        Serial.print(prompt);
        String s = wizReadLine();
        if (s.length() == 0) { Serial.println("  (rien recu, reessayez)"); continue; }
        uint8_t mask = 0;
        bool valid = true;
        int start = 0;
        for (int i = 0; i <= (int)s.length(); i++) {
            if (i == (int)s.length() || s[i] == '/') {
                String token = s.substring(start, i);
                token.trim();
                if (token.length() > 0) {
                    int v = token.toInt();
                    if (v >= 1 && v <= 3) mask |= (1u << (v - 1));
                    else { valid = false; break; }
                }
                start = i + 1;
            }
        }
        if (!valid || mask == 0) {
            Serial.println("  -> Format invalide. Ex: 1  ou  1/2  ou  2/3");
            continue;
        }
        Serial.printf("  -> %s\n", evMaskToStr(mask).c_str());
        return mask;
    }
}

// ============================================================
// SERIAL — RUNTIME (non-bloquant, utilisé dans loop())
// ============================================================
// Principe : on lit octet par octet dans le buffer hardware (64 bytes).
// Quand '\n' détecté → ligne complète → on la traite.
// Jamais de blocage, jamais de delay.

// Forward declarations
void processRuntimeCommand(const char* cmd);

void handleSerialRuntime() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue; // ignore CR
        if (c == '\n') {
            serialBuf[serialBufIdx] = '\0';
            if (serialBufIdx > 0) {
                processRuntimeCommand(serialBuf);
            }
            serialBufIdx = 0;
        } else if (serialBufIdx < sizeof(serialBuf) - 1) {
            serialBuf[serialBufIdx++] = c;
        } else {
            // Buffer plein → flush et ignore
            serialBufIdx = 0;
            Serial.println("[SERIAL] Ligne trop longue, ignoree.");
        }
    }
}

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void runTestMode();
void startWateringCycle(uint8_t valveMask, uint8_t zone, float zoneMaxL);
void stopWateringCycle();
SensorReading readSensorRaw(byte address);
void printStatus();

// ============================================================
// COMMANDES RUNTIME (depuis Serial ou MQTT — même parser)
// Format : "CMD" ou "CMD:valeur"
// ============================================================
void processRuntimeCommand(const char* raw) {
    // Copie pour pouvoir modifier
    char buf[64];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf)-1] = '\0';

    // Séparer cmd:valeur
    char* sep = strchr(buf, ':');
    char* cmd = buf;
    char* val = nullptr;
    if (sep) {
        *sep = '\0';
        val = sep + 1;
    }

    // Tout en majuscules pour comparaison insensible à la casse
    for (char* p = cmd; *p; p++) *p = toupper(*p);

    if (strcmp(cmd, "WATER") == 0 && val) {
        uint8_t zone = (uint8_t)atoi(val);
        if (zone == 1)      startWateringCycle(config.evMaskSensor1, 1, computeZoneMax(1));
        else if (zone == 2) startWateringCycle(config.evMaskSensor2, 2, computeZoneMax(2));
        else Serial.println("[CMD] Zone invalide (1=capteur1, 2=capteur2)");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        stopWateringCycle();
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        printStatus();
    }
    else if (strcmp(cmd, "RESET_DAILY") == 0) {
        setDailyUsedL(1, 0.0f);
        setDailyUsedL(2, 0.0f);
        Serial.println("[CMD] Compteurs journaliers remis a zero.");
    }
    else {
        Serial.printf("[CMD] Inconnu : '%s'. Cmds: WATER:1, WATER:2, STOP, STATUS, RESET_DAILY\n", raw);
    }
}

// Même entrée pour MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buf[64] = {0};
    memcpy(buf, payload, min((size_t)length, sizeof(buf) - 1));
    processRuntimeCommand(buf);
}

// ============================================================
// HELPERS
// ============================================================
static uint8_t countBits(uint8_t mask) {
    uint8_t n = 0;
    while (mask) { n += mask & 1u; mask >>= 1; }
    return n;
}

// Budget journalier d'une zone, proportionnel à son nombre d'EV.
// Ex: 3 EV total, zone1=2 EV → zone1 reçoit 2/3 du budget total.
static float computeZoneMax(uint8_t zone) {
    float    totalMax = 10.0f / (float)config.wateringDays;
    uint8_t  ev1      = countBits(config.evMaskSensor1);
    uint8_t  ev2      = countBits(config.evMaskSensor2);
    uint8_t  total    = ev1 + ev2;
    if (total == 0) return totalMax / 2.0f;
    return totalMax * (float)(zone == 1 ? ev1 : ev2) / (float)total;
}

// Vrai si on est dans une fenêtre d'arrosage automatique (6h ou 20h)
static bool isWateringWindow() {
    if (!rtcAvailable) return true;  // sans RTC: toujours actif
    uint8_t h = rtc.now().hour();
    return (h == 6 || h == 20);
}

// Vrai si on est dans la session du soir (20h+), utilisé pour emprunter budget Z2
static bool isEveningSession() {
    if (!rtcAvailable) return false;
    return rtc.now().hour() >= 20;
}

// Convertit un masque en chaîne lisible: 0b011 → "EV1/EV2"
static String evMaskToStr(uint8_t mask) {
    String s = "";
    for (uint8_t i = 0; i < 3; i++) {
        if (mask & (1u << i)) {
            if (s.length() > 0) s += "/";
            s += "EV" + String(i + 1);
        }
    }
    if (s.length() == 0) s = "none";
    return s;
}

// ============================================================
// GPIO / VALVES / PUMP
// ============================================================
void forceGPIOMode(uint8_t pin) {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}

void setGPIO(uint8_t pin, bool state) {
    gpio_set_level(static_cast<gpio_num_t>(pin), state ? 1 : 0);
}

void openValves(uint8_t mask) {
    const uint8_t pins[] = {cfg::EV1_PIN, cfg::EV2_PIN, cfg::EV3_PIN};
    for (uint8_t i = 0; i < 3; i++) {
        if (mask & (1u << i)) setGPIO(pins[i], true);
    }
    Serial.printf("%s OUVERTES\n", evMaskToStr(mask).c_str());
}

void closeValves(uint8_t mask) {
    const uint8_t pins[] = {cfg::EV1_PIN, cfg::EV2_PIN, cfg::EV3_PIN};
    for (uint8_t i = 0; i < 3; i++) {
        if (mask & (1u << i)) setGPIO(pins[i], false);
    }
}

void closeAllValves() {
    closeValves(0b111);
}

void activatePump() {
    setGPIO(cfg::PUMP_PIN, true);
    wateringState.pumpActive = true;
    Serial.println("Pompe ON");
}

void deactivatePump() {
    setGPIO(cfg::PUMP_PIN, false);
    wateringState.pumpActive = false;
    Serial.println("Pompe OFF");
}

// ============================================================
// MODBUS / SENSOR
// ============================================================
uint16_t modbusCRC(const byte* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x0001) ? ((crc >> 1U) ^ 0xA001) : (crc >> 1U);
    }
    return crc;
}

SensorReading readSensorRaw(byte address) {
    SensorReading reading;
    byte request[] = {address, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00};
    uint16_t crc = modbusCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    rs485Serial.write(request, sizeof(request));
    rs485Serial.flush();
    delay(150);
    int len = rs485Serial.readBytes(response, sizeof(response));
    if (len >= 7 && response[0] == address && response[1] == 0x03) {
        reading.humidity    = ((response[3] << 8) | response[4]) / 10.0f;
        reading.temperature = static_cast<int16_t>((response[5] << 8) | response[6]) / 10.0f;
        reading.ok = true;
    }
    return reading;
}

float rawToPercent(float raw, float dry, float wet) {
    if (wet == dry) return raw;
    float pct = (raw - dry) / (wet - dry) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

SensorReading readSensor(byte address, float dry, float wet, bool verbose = false) {
    SensorReading r = readSensorRaw(address);
    if (r.ok) {
        r.humidity = rawToPercent(r.humidity, dry, wet);
        if (verbose)
            Serial.printf("Sensor 0x%02X -> H:%.1f%% T:%.1fC\n", address, r.humidity, r.temperature);
    } else if (verbose) {
        Serial.printf("Sensor 0x%02X -> pas de reponse\n", address);
    }
    return r;
}

// ============================================================
// STATUS
// ============================================================
void printStatus() {
    Serial.println("=== STATUS ===");
    if (rtcAvailable) {
        DateTime now = rtc.now();
        Serial.printf("Heure: %04d/%02d/%02d %02d:%02d:%02d\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    }
    Serial.printf("Pompe: %s\n", wateringState.pumpActive ? "ON" : "OFF");
    if (wateringState.pumpActive)
        Serial.printf("Valves: %s | temps restant: %lus\n",
            evMaskToStr(wateringState.activeValveMask).c_str(),
            (wateringState.maxDuration - (millis() - wateringState.cycleStartTime)) / 1000);
    float zm1 = computeZoneMax(1), zm2 = computeZoneMax(2);
    Serial.printf("Budget Z1(%s): %.2f/%.2fL  Z2(%s): %.2f/%.2fL\n",
        evMaskToStr(config.evMaskSensor1).c_str(), getDailyUsedL(1), zm1,
        evMaskToStr(config.evMaskSensor2).c_str(), getDailyUsedL(2), zm2);
    Serial.println("==============");
}

// ============================================================
// TEST MODE
// ============================================================
void runTestMode() {
    Serial.println("Test: amorcage pompe 15s, puis EV1->EV2->EV3 x 5s chacune");
    closeAllValves();
    activatePump();
    delay(cfg::PRIME_DURATION_MS);  // remplissage des tuyaux
    for (uint8_t v = 1; v <= 3; v++) {
        Serial.printf("  EV%d...\n", v);
        openValves(1u << (v - 1));
        delay(5000);
        closeAllValves();
        delay(500);
    }
    deactivatePump();
    Serial.println("Test termine.");
}

// ============================================================
// CALIBRATION (appelée depuis le wizard, peut retourner false si skip)
// ============================================================
bool calibrateSensor(const char* name, byte address, float& dry, float& wet) {
    Serial.printf("\n--- Calibration %s ---\n", name);

    // Étape 1 : sec
    while (true) {
        Serial.print("Etape 1: capteur a l'air SEC. Tape 'o' (ok), 'p' (passer), 'r' (recommencer) : ");
        char c = wizReadChar();
        if (c == 'p') { Serial.println("Passe."); return false; }
        if (c == 'o') {
            SensorReading r = readSensorRaw(address);
            if (!r.ok) { Serial.println("Capteur absent. Valeurs par defaut (0/100)."); dry = 0.0f; wet = 100.0f; return false; }
            dry = r.humidity;
            Serial.printf("  Sec: %.1f\n", dry);
            break;
        }
        // 'r' ou autre → recommencer
    }

    // Étape 2 : humide
    while (true) {
        Serial.print("Etape 2: capteur dans l'eau. Tape 'o' (ok), 'r' (recommencer) : ");
        char c = wizReadChar();
        if (c == 'o') {
            SensorReading r = readSensorRaw(address);
            if (!r.ok) { Serial.println("Capteur absent. Valeur humide inchangee."); return true; }
            wet = r.humidity;
            Serial.printf("  Humide: %.1f\n", wet);
            break;
        }
        // 'r' → recommencer étape 2
    }

    Serial.printf("Calib %s: sec=%.1f humide=%.1f\n", name, dry, wet);
    return true;
}

// ============================================================
// WIZARD DE CONFIGURATION
// ============================================================
// Pas de setjmp/longjmp. Pas de goto.
// Un bool `restart` permet de recommencer proprement.
// ============================================================
void runCalibrationWizard() {
    bool restart = true;

    while (restart) {
        restart = false;
        clearSerial();

        Serial.println("\n========================================");
        Serial.println(" ASSISTANT DE CONFIGURATION");
        Serial.println(" Tape 'r' n'importe quand pour recommencer");
        Serial.println("========================================");

        // ----- Heure RTC -----
        if (rtcAvailable) {
            bool lostPower = rtc.lostPower();
            bool timeOk = false;
            while (!timeOk) {
                DateTime now = rtc.now();
                Serial.printf("\nHeure: %04d/%02d/%02d %02d:%02d:%02d",
                    now.year(), now.month(), now.day(),
                    now.hour(), now.minute(), now.second());
                if (lostPower) Serial.print("  [BATTERIE RTC vide!]");
                Serial.println();
                Serial.println("  o = heure correcte");
                Serial.println("  c = regler a l'heure de compilation");
                Serial.println("  m = saisir manuellement");
                Serial.print("Choix : ");
                char ch = wizReadChar();
                if (ch == 'o') {
                    timeOk = true;
                } else if (ch == 'c') {
                    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
                    Serial.println("Regle sur l'heure de compilation.");
                    lostPower = false;
                } else if (ch == 'm') {
                    int yr = wizReadInt("Annee (ex: 2026) : ", 2024, 2100);
                    int mo = wizReadInt("Mois (1-12) : ", 1, 12);
                    int dy = wizReadInt("Jour (1-31) : ", 1, 31);
                    int hr = wizReadInt("Heure (0-23) : ", 0, 23);
                    int mn = wizReadInt("Minute (0-59) : ", 0, 59);
                    rtc.adjust(DateTime(yr, mo, dy, hr, mn, 0));
                    lostPower = false;
                }
            }
        } else {
            Serial.println("\nRTC non trouvee - fonctionnement sans horloge.");
        }

        // ----- Calibration -----
        if (wizAskON("\nCalibrer les capteurs maintenant")) {
            calibrateSensor("Capteur 1", cfg::SENSOR1_ADDR, calibData.s1Dry, calibData.s1Wet);
            calibrateSensor("Capteur 2", cfg::SENSOR2_ADDR, calibData.s2Dry, calibData.s2Wet);
        } else {
            Serial.println("Valeurs par defaut utilisees.");
        }

        // ----- Tuyaux OK -----
        {
            bool plumbingOk = false;
            while (!plumbingOk) {
                Serial.print("\nCapteurs et tuyaux bien mis ? 'o' pour continuer, 'r' pour recommencer : ");
                char c = wizReadChar();
                if (c == 'r') { restart = true; break; }
                if (c == 'o') plumbingOk = true;
            }
            if (restart) continue;
        }

        // ----- Association capteur <-> EV -----
        {
            bool evOk = false;
            while (!evOk) {
                Serial.println("\n--- Association capteurs <-> electrovannes ---");
                Serial.println("  Entrer 1, 2, 3 ou combinaison ex: 1/2");
                uint8_t mask1 = wizReadEVMask("Capteur 1 -> EV(s) : ");
                uint8_t mask2 = wizReadEVMask("Capteur 2 -> EV(s) : ");
                Serial.printf("  S1->%s  S2->%s\n",
                    evMaskToStr(mask1).c_str(), evMaskToStr(mask2).c_str());
                Serial.print("Confirmer (o=ok, r=recommencer association) : ");
                char c = wizReadChar();
                if (c == 'o') {
                    config.evMaskSensor1 = mask1;
                    config.evMaskSensor2 = mask2;
                    evOk = true;
                }
                // tout autre char → recommencer l'association (pas le wizard complet)
            }
        }

        // ----- Nombre de jours -----
        {
            bool daysOk = false;
            while (!daysOk) {
                int days = wizReadInt("\nNombre de jours d'arrosage (reservoir 10 L) [1-30] : ", 1, 30);
                Serial.printf("  Max/jour: %.2f L\n", 10.0f / (float)days);
                Serial.print("Confirmer (o/r) : ");
                char c = wizReadChar();
                if (c == 'o') {
                    config.wateringDays = (uint8_t)days;
                    daysOk = true;
                }
            }
        }

        // ----- Test -----
        if (wizAskON("\nLancer le mode test (EV1/EV2/EV3 x 5s)")) {
            runTestMode();
            bool testOk = false;
            while (!testOk) {
                Serial.print("Tout fonctionne ? (o=continuer, r=recommencer wizard) : ");
                char c = wizReadChar();
                if (c == 'o') testOk = true;
                if (c == 'r') { restart = true; break; }
            }
            if (restart) continue;
        }

        // Wizard terminé
        saveNVS();
        Serial.println("\nConfiguration sauvegardee. Demarrage normal...\n");
    }
}

// ============================================================
// WATERING
// ============================================================
float litersForDuration(unsigned long ms) {
    return (ms / 1000.0f) / 60.0f * 10.0f;
}

void startWateringCycle(uint8_t valveMask, uint8_t zone, float zoneMaxL) {
    if (valveMask == 0 || valveMask > 0b111 || zone < 1 || zone > 2) return;
    if (wateringState.pumpActive) { Serial.println("Cycle deja actif."); return; }

    float used = getDailyUsedL(zone);
    if (used >= zoneMaxL) {
        Serial.printf("Budget Z%d atteint (%.2f/%.2fL).\n", zone, used, zoneMaxL);
        return;
    }

    unsigned long now_ms = millis();
    for (uint8_t i = 0; i < 3; i++) {
        if ((valveMask & (1u << i)) &&
            wateringState.lastCycleTime[i] > 0 &&
            now_ms - wateringState.lastCycleTime[i] < cfg::COOLDOWN_MS) {
            Serial.println("Cooldown actif, arrosage ignore.");
            return;
        }
    }

    float remaining = zoneMaxL - used;
    unsigned long maxDur = (unsigned long)((remaining / 10.0f) * 60.0f * 1000.0f);
    unsigned long duration = min(cfg::WATER_DUR_MS, maxDur);

    Serial.printf("Arrosage Z%d %s pendant %lus (reste %.2fL)\n",
        zone, evMaskToStr(valveMask).c_str(), duration / 1000, remaining);
    closeAllValves();
    delay(100);
    // Pompe en premier pour remplir les tuyaux ; les EV s'ouvrent après PRIME_DURATION_MS
    activatePump();
    wateringState.activeValveMask  = valveMask;
    wateringState.activeZone       = zone;
    wateringState.maxDuration      = duration;
    wateringState.isPriming        = true;
    wateringState.primingStartTime = millis();
    wateringState.cycleStartTime   = 0;  // sera fixé quand les EV s'ouvrent
    Serial.printf("Amorcage pompe %lus avant ouverture EV...\n", cfg::PRIME_DURATION_MS / 1000);
    for (uint8_t i = 0; i < 3; i++) {
        if (valveMask & (1u << i)) wateringState.lastCycleTime[i] = now_ms;
    }
}

void stopWateringCycle() {
    if (!wateringState.pumpActive) return;
    uint8_t z = wateringState.activeZone;
    if (!wateringState.isPriming && wateringState.cycleStartTime > 0) {
        // Phase d'arrosage active : comptabiliser le budget
        unsigned long actual = millis() - wateringState.cycleStartTime;
        unsigned long capped = min(actual, wateringState.maxDuration);
        float liters = litersForDuration(capped);
        setDailyUsedL(z, getDailyUsedL(z) + liters);
        Serial.printf("Cycle stoppe. %.2fL utilises (Z%d).\n", liters, z);
    } else {
        Serial.println("Cycle stoppe pendant amorcage (budget inchange).");
    }
    deactivatePump();
    closeAllValves();
    wateringState.activeValveMask  = 0;
    wateringState.activeZone       = 0;
    wateringState.cycleStartTime   = 0;
    wateringState.isPriming        = false;
    wateringState.primingStartTime = 0;
    pendingPostWateringPublish     = true;
}

void checkWateringCycleTimeout() {
    if (!wateringState.pumpActive || wateringState.activeValveMask == 0) return;

    if (wateringState.isPriming) {
        // Phase d'amorçage : ouvrir les EV après PRIME_DURATION_MS
        if (millis() - wateringState.primingStartTime >= cfg::PRIME_DURATION_MS) {
            openValves(wateringState.activeValveMask);
            wateringState.isPriming      = false;
            wateringState.cycleStartTime = millis();
            Serial.println("EV ouvertes, arrosage demarre.");
        }
        return;  // pas encore en phase d'arrosage
    }

    if (millis() - wateringState.cycleStartTime >= wateringState.maxDuration) {
        Serial.println("Cycle termine (timeout).");
        stopWateringCycle();
    }
}

void checkAutomaticWatering(const SensorReading& s1, const SensorReading& s2) {
    if (wateringState.pumpActive) return;
    if (!isWateringWindow()) return;  // uniquement à 6h et 20h

    float zm1    = computeZoneMax(1);
    float zm2    = computeZoneMax(2);
    float used1  = getDailyUsedL(1);
    float used2  = getDailyUsedL(2);
    float avail1 = zm1 - used1;
    float avail2 = zm2 - used2;

    // ----- Zone 1 -----
    if (s1.ok && s1.humidity < cfg::HUMIDITY_THRESHOLD) {
        if (avail1 > 0) {
            Serial.printf("S1 bas (%.1f%%) -> %s\n",
                s1.humidity, evMaskToStr(config.evMaskSensor1).c_str());
            startWateringCycle(config.evMaskSensor1, 1, zm1);
            return;
        }
        // Budget Z1 épuisé. Le soir, emprunter le budget non utilisé de Z2
        bool z2Satisfied = !s2.ok || s2.humidity >= cfg::HUMIDITY_THRESHOLD;
        if (isEveningSession() && z2Satisfied && avail2 > 0) {
            Serial.printf("S1 bas (%.1f%%), emprunte %.2fL de Z2 (soir)\n",
                s1.humidity, avail2);
            startWateringCycle(config.evMaskSensor1, 1, zm1 + avail2);
            return;
        }
        // Vraiment à sec
        if (millis() - lastAlertTime >= 600000UL) {
            lastAlertTime = millis();
            char msg[64];
            bool reservoirVide = (avail1 <= 0 && avail2 <= 0);
            if (reservoirVide)
                snprintf(msg, sizeof(msg), "Reservoir vide - Z1(%s) a soif",
                    evMaskToStr(config.evMaskSensor1).c_str());
            else
                snprintf(msg, sizeof(msg), "Z1(%s) a soif, budget epuise",
                    evMaskToStr(config.evMaskSensor1).c_str());
            publishAlert(msg);
        }
    }

    // ----- Zone 2 (vérifiée même si Z1 a épuisé son budget) -----
    if (s2.ok && s2.humidity < cfg::HUMIDITY_THRESHOLD) {
        if (avail2 > 0) {
            Serial.printf("S2 bas (%.1f%%) -> %s\n",
                s2.humidity, evMaskToStr(config.evMaskSensor2).c_str());
            startWateringCycle(config.evMaskSensor2, 2, zm2);
        } else {
            if (millis() - lastAlertTime >= 600000UL) {
                lastAlertTime = millis();
                char msg[64];
                snprintf(msg, sizeof(msg), "Z2(%s) a soif, budget epuise",
                    evMaskToStr(config.evMaskSensor2).c_str());
                publishAlert(msg);
            }
        }
    }
}

// ============================================================
// DISPLAY
// ============================================================
void updateDisplay(const SensorReading& s1, const SensorReading& s2) {
    display.clearDisplay();
    display.setCursor(0, 0);
    if (s1.ok) display.printf("S1 H:%.1f%% T:%.1fC", s1.humidity, s1.temperature);
    else        display.print("S1: no response");
    display.setCursor(0, 8);
    if (s2.ok) display.printf("S2 H:%.1f%% T:%.1fC", s2.humidity, s2.temperature);
    else        display.print("S2: no response");
    display.display();
}

// ============================================================
// WIFI / MQTT
// ============================================================
void startAccessPoint() {
    static bool started = false;
    if (started) return;
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(cfg::AP_LOCAL_IP, cfg::AP_GATEWAY, cfg::AP_SUBNET);
    if (cfg::AP_PASSWORD_LEN >= 8) WiFi.softAP(cfg::AP_SSID, cfg::AP_PASSWORD);
    else                            WiFi.softAP(cfg::AP_SSID);
    started = true;
    Serial.printf("SoftAP: %s  IP: %s\n", cfg::AP_SSID, WiFi.softAPIP().toString().c_str());
}

void tryConnectMqtt() {
    if (mqttClient.connected()) return;
    unsigned long now = millis();
    if (now - lastMqttAttempt < 30000UL) return;  // une tentative toutes les 30 s
    lastMqttAttempt = now;
    mqttClient.setServer(cfg::MQTT_HOST, cfg::MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    if (mqttClient.connect(cfg::MQTT_CLIENT_ID)) {
        Serial.println("MQTT connecte.");
        mqttClient.subscribe(cfg::MQTT_CMD_TOPIC);
    } else {
        Serial.printf("MQTT echec (rc=%d), prochaine tentative dans 30s.\n", mqttClient.state());
    }
}

void publishSensorReadings(uint8_t idx, const SensorReading& r) {
    if (!r.ok || !mqttClient.connected()) return;
    char topic[64], payload[16];
    snprintf(topic,   sizeof(topic),   "garden/sensors/sensor%u/temperature", idx);
    snprintf(payload, sizeof(payload), "%.1f", r.temperature);
    mqttClient.publish(topic, payload, true);
    snprintf(topic,   sizeof(topic),   "garden/sensors/sensor%u/humidity", idx);
    snprintf(payload, sizeof(payload), "%.1f", r.humidity);
    mqttClient.publish(topic, payload, true);
}

void publishPumpState(bool state) {
    if (!mqttClient.connected()) return;
    mqttClient.publish("garden/pump/state", state ? "ON" : "OFF", true);
}

// Alerte "Soif" — publiée sur garden/alert/soif
void publishAlert(const char* msg) {
    Serial.printf("[ALERTE] %s\n", msg);
    if (!mqttClient.connected()) return;
    mqttClient.publish("garden/alert/soif", msg, false);
}

// Publié juste après la fin d'un cycle d'arrosage
void publishPostWateringStatus() {
    SensorReading ps1 = readSensor(cfg::SENSOR1_ADDR, calibData.s1Dry, calibData.s1Wet, true);
    delay(200);
    SensorReading ps2 = readSensor(cfg::SENSOR2_ADDR, calibData.s2Dry, calibData.s2Wet, true);
    float zm1 = computeZoneMax(1), zm2 = computeZoneMax(2);
    Serial.printf("Budget restant: Z1 %.2f/%.2fL  Z2 %.2f/%.2fL\n",
        getDailyUsedL(1), zm1, getDailyUsedL(2), zm2);
    if (!mqttClient.connected()) return;
    publishSensorReadings(1, ps1);
    publishSensorReadings(2, ps2);
    publishPumpState(false);
    char payload[64];
    snprintf(payload, sizeof(payload), "Z1:%.2f/%.2fL Z2:%.2f/%.2fL",
        getDailyUsedL(1), zm1, getDailyUsedL(2), zm2);
    mqttClient.publish("garden/water/budget", payload, true);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    SPI.end();
    Serial.begin(115200);

    // *** CLEF : timeout long pour le wizard (attente humaine).
    // Dans loop(), on n'appelle readStringUntil() donc ce timeout
    // n'a aucun impact sur le fonctionnement normal.
    Serial.setTimeout(30000);
    delay(500);

    // EV3_PIN (GPIO3) = UART0 RX : ne pas le configurer en sortie avant le wizard
    // sinon Serial.available() retourne toujours 0 (pin tiré bas).
    // EV3 est initialisé APRÈS le wizard (voir plus bas).
    forceGPIOMode(cfg::PUMP_PIN);
    forceGPIOMode(cfg::EV1_PIN);
    forceGPIOMode(cfg::EV2_PIN);
    setGPIO(cfg::PUMP_PIN, false);
    setGPIO(cfg::EV1_PIN,  false);
    setGPIO(cfg::EV2_PIN,  false);

    rs485Serial.begin(4800, SERIAL_8N1, 16, 17);
    rs485Serial.setTimeout(300);

    Wire.begin(cfg::OLED_SDA, cfg::OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED init failed");
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Demarrage...");
    display.display();

    rtcWire.begin(cfg::RTC_SDA, cfg::RTC_SCL);
    rtcAvailable = rtc.begin(&rtcWire);
    if (!rtcAvailable) Serial.println("ATTENTION: RTC DS3231 non trouvee!");

    loadNVS();

    startAccessPoint();
    runCalibrationWizard();

    // Wizard terminé → configurer EV3 (GPIO3) en sortie maintenant que
    // la réception série n'est plus nécessaire pour le setup.
    forceGPIOMode(cfg::EV3_PIN);
    setGPIO(cfg::EV3_PIN, false);

    clearSerial();

    tryConnectMqtt();

    Serial.println("Pret. Commandes: WATER:1 (capteur1), WATER:2 (capteur2), STOP, STATUS, RESET_DAILY");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // 1. Commandes série (non-bloquant — char par char)
    handleSerialRuntime();

    // 2. MQTT
    if (!mqttClient.connected()) tryConnectMqtt();
    mqttClient.loop();

    // 3. Timeout arrosage (vérifié à chaque itération)
    checkWateringCycleTimeout();

    // 3b. Publication post-arrosage (déclenché par stopWateringCycle)
    if (pendingPostWateringPublish) {
        pendingPostWateringPublish = false;
        publishPostWateringStatus();
    }

    // 4. Remise à zéro journalière via RTC (toutes les 60 s)
    if (now - lastDailyCheck >= 60000UL) {
        lastDailyCheck = now;
        resetDailyUsedIfNewDay();
    }

    // 5. Lecture capteurs toutes les 30 s
    if (now - lastSensorRead >= cfg::SENSOR_READ_MS) {
        lastSensorRead = now;
        bool verbose = (now - lastSensorLog >= cfg::SENSOR_LOG_MS);
        if (verbose) lastSensorLog = now;

        SensorReading s1 = readSensor(cfg::SENSOR1_ADDR, calibData.s1Dry, calibData.s1Wet, verbose);
        delay(200);
        SensorReading s2 = readSensor(cfg::SENSOR2_ADDR, calibData.s2Dry, calibData.s2Wet, verbose);

        checkAutomaticWatering(s1, s2);
        updateDisplay(s1, s2);

        if (now - lastPublish >= cfg::PUBLISH_MS) {
            publishSensorReadings(1, s1);
            publishSensorReadings(2, s2);
            publishPumpState(wateringState.pumpActive);
            lastPublish = now;
        }
    }

    delay(100);
}
