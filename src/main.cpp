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
    constexpr unsigned long PUBLISH_MS    = 10000UL;

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

    constexpr float    HUMIDITY_THRESHOLD = 30.0f;
    constexpr unsigned long WATER_DUR_MS  = 30000UL;
    constexpr unsigned long COOLDOWN_MS   = 120000UL;
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
    uint8_t evForSensor1 = 1;
    uint8_t evForSensor2 = 3;
    uint8_t wateringDays = 10;
};

struct WateringState {
    bool          pumpActive               = false;
    uint8_t       activeValve              = 0;
    unsigned long cycleStartTime           = 0;
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

byte          response[20];
unsigned long lastPublish = 0;
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
constexpr char NVS_EV_MAP[]     = "ev_map";
constexpr char NVS_WATER_DAYS[] = "water_days";
constexpr char NVS_DAILY_USED[] = "daily_used";
constexpr char NVS_LAST_DAY[]   = "last_day";

void loadNVS() {
    prefs.begin(NVS_NS, true);
    calibData.s1Dry     = prefs.getFloat(NVS_S1_DRY, 0.0f);
    calibData.s1Wet     = prefs.getFloat(NVS_S1_WET, 100.0f);
    calibData.s2Dry     = prefs.getFloat(NVS_S2_DRY, 0.0f);
    calibData.s2Wet     = prefs.getFloat(NVS_S2_WET, 100.0f);
    uint8_t evMap       = prefs.getUChar(NVS_EV_MAP, 0b00001101);
    config.evForSensor1 = (evMap & 0x03);
    config.evForSensor2 = ((evMap >> 2) & 0x03);
    config.wateringDays = prefs.getUChar(NVS_WATER_DAYS, 10);
    prefs.end();
}

void saveNVS() {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(NVS_S1_DRY,    calibData.s1Dry);
    prefs.putFloat(NVS_S1_WET,    calibData.s1Wet);
    prefs.putFloat(NVS_S2_DRY,    calibData.s2Dry);
    prefs.putFloat(NVS_S2_WET,    calibData.s2Wet);
    uint8_t evMap = (config.evForSensor1 & 0x03) | ((config.evForSensor2 & 0x03) << 2);
    prefs.putUChar(NVS_EV_MAP,    evMap);
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

float getDailyUsedL() {
    prefs.begin(NVS_NS, true);
    float v = prefs.getFloat(NVS_DAILY_USED, 0.0f);
    prefs.end();
    return v;
}

void setDailyUsedL(float v) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(NVS_DAILY_USED, v);
    prefs.end();
}

void resetDailyUsedIfNewDay(uint8_t today) {
    prefs.begin(NVS_NS, false);
    uint8_t last = prefs.getUChar(NVS_LAST_DAY, 0);
    if (last != today) {
        prefs.putFloat(NVS_DAILY_USED, 0.0f);
        prefs.putUChar(NVS_LAST_DAY, today);
    }
    prefs.end();
}

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

// ============================================================
// SERIAL — RUNTIME (non-bloquant, utilisé dans loop())
// ============================================================
// Principe : on lit octet par octet dans le buffer hardware (64 bytes).
// Quand '\n' détecté → ligne complète → on la traite.
// Jamais de blocage, jamais de delay.

// Forward declaration
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
void startWateringCycle(uint8_t valveNumber, float maxL);
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

    float dailyMax = 10.0f / (float)config.wateringDays;

    if (strcmp(cmd, "WATER") == 0 && val) {
        uint8_t v = (uint8_t)atoi(val);
        if (v >= 1 && v <= 3) startWateringCycle(v, dailyMax);
        else Serial.println("[CMD] Valve invalide (1-3)");
    }
    else if (strcmp(cmd, "STOP") == 0) {
        stopWateringCycle();
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        printStatus();
    }
    else if (strcmp(cmd, "RESET_DAILY") == 0) {
        setDailyUsedL(0.0f);
        Serial.println("[CMD] Compteur journalier remis a zero.");
    }
    else {
        Serial.printf("[CMD] Inconnu : '%s'. Cmds: WATER:1, STOP, STATUS, RESET_DAILY\n", raw);
    }
}

// Même entrée pour MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buf[64] = {0};
    memcpy(buf, payload, min((size_t)length, sizeof(buf) - 1));
    processRuntimeCommand(buf);
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

void openValve(uint8_t n) {
    const uint8_t pins[] = {cfg::EV1_PIN, cfg::EV2_PIN, cfg::EV3_PIN};
    if (n < 1 || n > 3) return;
    setGPIO(pins[n-1], true);
    Serial.printf("Valve %d OUVERTE\n", n);
}

void closeValve(uint8_t n) {
    const uint8_t pins[] = {cfg::EV1_PIN, cfg::EV2_PIN, cfg::EV3_PIN};
    if (n < 1 || n > 3) return;
    setGPIO(pins[n-1], false);
    Serial.printf("Valve %d FERMEE\n", n);
}

void closeAllValves() {
    for (uint8_t i = 1; i <= 3; i++) closeValve(i);
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

SensorReading readSensor(byte address, float dry, float wet) {
    SensorReading r = readSensorRaw(address);
    if (r.ok) {
        r.humidity = rawToPercent(r.humidity, dry, wet);
        Serial.printf("Sensor 0x%02X -> H:%.1f%% T:%.1fC\n", address, r.humidity, r.temperature);
    } else {
        Serial.printf("Sensor 0x%02X -> pas de reponse\n", address);
    }
    return r;
}

// ============================================================
// STATUS
// ============================================================
void printStatus() {
    Serial.println("=== STATUS ===");
    Serial.printf("Pompe: %s\n", wateringState.pumpActive ? "ON" : "OFF");
    if (wateringState.pumpActive)
        Serial.printf("Valve active: EV%d | temps restant: %lus\n",
            wateringState.activeValve,
            (wateringState.maxDuration - (millis() - wateringState.cycleStartTime)) / 1000);
    Serial.printf("Budget journalier: %.2f / %.2f L\n",
        getDailyUsedL(), 10.0f / (float)config.wateringDays);
    Serial.printf("S1->EV%d  S2->EV%d\n", config.evForSensor1, config.evForSensor2);
    Serial.println("==============");
}

// ============================================================
// TEST MODE
// ============================================================
void runTestMode() {
    Serial.println("Test: EV1->EV2->EV3, 5s chacune");
    for (uint8_t v = 1; v <= 3; v++) {
        closeAllValves();
        delay(200);
        openValve(v);
        activatePump();
        delay(5000);
        deactivatePump();
        closeAllValves();
        delay(500);
    }
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
                int ev1 = wizReadInt("Capteur 1 -> EV numero (1/2/3) : ", 1, 3);
                int ev2 = wizReadInt("Capteur 2 -> EV numero (1/2/3) : ", 1, 3);
                Serial.printf("  S1->EV%d  S2->EV%d\n", ev1, ev2);
                Serial.print("Confirmer (o=ok, r=recommencer association) : ");
                char c = wizReadChar();
                if (c == 'o') {
                    config.evForSensor1 = (uint8_t)ev1;
                    config.evForSensor2 = (uint8_t)ev2;
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

void startWateringCycle(uint8_t valveNumber, float maxL) {
    if (valveNumber < 1 || valveNumber > 3) return;
    if (wateringState.pumpActive) { Serial.println("Cycle deja actif."); return; }

    float used = getDailyUsedL();
    if (used >= maxL) {
        Serial.printf("Budget atteint (%.2f/%.2f L).\n", used, maxL);
        return;
    }

    unsigned long now_ms = millis();
    if (wateringState.lastCycleTime[valveNumber-1] > 0 &&
        now_ms - wateringState.lastCycleTime[valveNumber-1] < cfg::COOLDOWN_MS) {
        Serial.println("Cooldown actif, arrosage ignore.");
        return;
    }

    float remaining = maxL - used;
    unsigned long maxDur = (unsigned long)((remaining / 10.0f) * 60.0f * 1000.0f);
    unsigned long duration = min(cfg::WATER_DUR_MS, maxDur);

    Serial.printf("Arrosage EV%d pendant %lus (reste %.2fL)\n",
        valveNumber, duration/1000, remaining);
    closeAllValves();
    delay(100);
    openValve(valveNumber);
    activatePump();
    wateringState.activeValve                  = valveNumber;
    wateringState.cycleStartTime               = millis();
    wateringState.maxDuration                  = duration;
    wateringState.lastCycleTime[valveNumber-1] = now_ms;
}

void stopWateringCycle() {
    if (!wateringState.pumpActive) return;
    float liters = litersForDuration(millis() - wateringState.cycleStartTime);
    setDailyUsedL(getDailyUsedL() + liters);
    Serial.printf("Cycle stoppe. %.2fL utilises.\n", liters);
    deactivatePump();
    closeAllValves();
    wateringState.activeValve    = 0;
    wateringState.cycleStartTime = 0;
}

void checkWateringCycleTimeout() {
    if (!wateringState.pumpActive || wateringState.activeValve == 0) return;
    if (millis() - wateringState.cycleStartTime >= wateringState.maxDuration) {
        Serial.println("Cycle termine (timeout).");
        stopWateringCycle();
    }
}

void checkAutomaticWatering(const SensorReading& s1, const SensorReading& s2) {
    if (wateringState.pumpActive) return;
    float dailyMax = 10.0f / (float)config.wateringDays;
    if (s1.ok && s1.humidity < cfg::HUMIDITY_THRESHOLD) {
        Serial.printf("S1 bas (%.1f%%) -> EV%d\n", s1.humidity, config.evForSensor1);
        startWateringCycle(config.evForSensor1, dailyMax);
        return;
    }
    if (s2.ok && s2.humidity < cfg::HUMIDITY_THRESHOLD) {
        Serial.printf("S2 bas (%.1f%%) -> EV%d\n", s2.humidity, config.evForSensor2);
        startWateringCycle(config.evForSensor2, dailyMax);
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
    mqttClient.setServer(cfg::MQTT_HOST, cfg::MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    if (mqttClient.connect(cfg::MQTT_CLIENT_ID)) {
        Serial.println("MQTT connecte.");
        mqttClient.subscribe(cfg::MQTT_CMD_TOPIC);
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

    loadNVS();
    setDailyUsedL(0.0f);

    startAccessPoint();
    runCalibrationWizard();

    // Wizard terminé → configurer EV3 (GPIO3) en sortie maintenant que
    // la réception série n'est plus nécessaire pour le setup.
    forceGPIOMode(cfg::EV3_PIN);
    setGPIO(cfg::EV3_PIN, false);

    clearSerial();

    tryConnectMqtt();

    Serial.println("Pret. Commandes: WATER:1, WATER:2, WATER:3, STOP, STATUS, RESET_DAILY");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // 1. Commandes série (non-bloquant — char par char)
    handleSerialRuntime();

    // 2. MQTT
    if (!mqttClient.connected()) tryConnectMqtt();
    mqttClient.loop();

    // 3. Timeout arrosage
    checkWateringCycleTimeout();

    // 4. Lecture capteurs
    SensorReading s1 = readSensor(cfg::SENSOR1_ADDR, calibData.s1Dry, calibData.s1Wet);
    delay(200);
    SensorReading s2 = readSensor(cfg::SENSOR2_ADDR, calibData.s2Dry, calibData.s2Wet);

    // 5. Arrosage automatique
    checkAutomaticWatering(s1, s2);

    // 6. Affichage
    updateDisplay(s1, s2);

    // 7. Publication MQTT périodique
    unsigned long now_ms = millis();
    if (now_ms - lastPublish >= cfg::PUBLISH_MS) {
        publishSensorReadings(1, s1);
        publishSensorReadings(2, s2);
        publishPumpState(wateringState.pumpActive);
        lastPublish = now_ms;
    }

    delay(1000);
}