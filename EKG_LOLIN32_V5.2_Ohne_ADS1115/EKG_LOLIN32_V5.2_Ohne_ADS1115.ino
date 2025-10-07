// =============================
// Include Required Libraries
// =============================
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_sleep.h>
#include "driver/gpio.h"

// =====================
// Configuration Macros
// =====================
//#define SAMPLE_RATE 250  // ECG sampling rate (Hz)
#define ENABLE_WIFI_MQTT
#define deviceNumber 1 // define device Number

// ============================
// WiFi & MQTT Configuration
// ============================
#ifdef ENABLE_WIFI_MQTT
#define WIFI_SSID "DimaAlpha"
#define WIFI_PASSWORD "Qwerty**21***79"
//#define MQTT_BROKER "192.168.178.44" // Pi 2
#define MQTT_BROKER "192.168.178.104" // Pi 4
//#define WIFI_SSID "EKG_Wlan"
//#define WIFI_PASSWORD "EKG_2025"
//#define MQTT_BROKER "192.168.1.101" //EKG_WLAN
#define MQTT_PORT 1883
String mqttTopic = "ecg/device" + String(deviceNumber);
WiFiClient espClient;
PubSubClient client(espClient);
#endif

// ==================
// Global Variables
// ==================

TaskHandle_t ledFlasherHandle;
unsigned long lastSampleMicros = 0;
unsigned long startTimestamp = 0;
unsigned long lastStatusSentMillis = 0;
const unsigned long statusIntervalMillis = 30000; // 30 seconds

unsigned long lastNtpSyncMillis = 0;
const unsigned long ntpSyncInterval = 10L * 60L * 1000L;
time_t startUtcTimestamp = 0;

bool setupComplete = false;

// Retry counters for connection attempts
int wifiRetries = 0;
int mqttRetries = 0;
const int MAX_RETRIES = 10;

// ============================
// Pin & Hardware Definitions
// ============================
#define BUTTON_PIN 0
#define LoPlus 16
#define LoMinus 17
#define SDN_PIN 33
#define Power_LED 32
#define VBATPIN 35
#define VBAT_DIVIDER 2.1 // Voltage divider factor influences battery percentage%
#define EKG_INPUT 34
#define SAMPLE_RATE 250 // desired sampling rate in Hz
const unsigned long SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE;

#define VBAT_MIN 3.3
#define VBAT_MAX 4.1
#define BATTERY_SMOOTHING 10
#define ENABLE_STATUS_SENDING true  // Disable sending of status. to increase sampling rate

// =====================
// Filter Coefficients
// =====================
float hp_b[] = {0.995566, -1.991132, 0.995566};
float hp_a[] = {1.0, -1.991114, 0.991153};
float hp_x[3] = {0}, hp_y[3] = {0};

float lp_b[] = {0.206572, 0.413145, 0.206572};
float lp_a[] = {1.0, -0.369527, 0.195814};
float lp_x[3] = {0}, lp_y[3] = {0};

// ========================
// Setup Function
// ========================
void setup() {
    xTaskCreatePinnedToCore(ledFlasherTask, "LEDFLASH", 1000, NULL, 1, &ledFlasherHandle, 1);

    Serial.begin(115200);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(Power_LED, OUTPUT);
    pinMode(LoPlus, INPUT);
    pinMode(LoMinus, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(SDN_PIN, OUTPUT);

    pinMode(EKG_INPUT, INPUT);
    gpio_hold_dis(GPIO_NUM_33);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke up from deep sleep.");
        digitalWrite(Power_LED, HIGH);
        digitalWrite(SDN_PIN, HIGH);
    } else {
        Serial.println("Powering on normally.");
        digitalWrite(Power_LED, LOW);
        digitalWrite(SDN_PIN, LOW);
    }

    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(BUTTON_PIN), 0);

#ifdef ENABLE_WIFI_MQTT
    if (!connectToWiFi()) enterDeepSleep();
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setBufferSize(1024);
    if (!connectToMQTT()) enterDeepSleep();

    configTime(0, 0, "pool.ntp.org");
    Serial.println("Warte auf NTP-Zeit...");
    time_t now = time(nullptr);
    while (now < 100000) {
        delay(100);
        now = time(nullptr);
    }
    Serial.println("NTP-Zeit empfangen.");
    lastNtpSyncMillis = millis();
#endif

    // ============================
    // Battery pre-fill: call getBatteryPercentage 10 times
    // ============================
    Serial.println("Initial battery measurement...");
    for (int i = 0; i < 10; i++) {
        float bat = getBatteryPercentage();
        Serial.printf("Initial battery check %d: %.1f%%\n", i+1, bat);
        delay(100); // leicht verzögert
    }

    setupComplete = true;
    delay(100);
    digitalWrite(Power_LED, HIGH);
}

// ===========================================
// Main Loop - Sample & Send Live
// ===========================================
void loop() {
    bool padsConnected = (digitalRead(LoPlus) == LOW && digitalRead(LoMinus) == LOW);

    unsigned long now = micros();
    digitalWrite(SDN_PIN, HIGH);

if (startTimestamp == 0) {
    startTimestamp = micros();
    startUtcTimestamp = time(nullptr);
}

 now = micros();
if (now - lastSampleMicros >= SAMPLE_INTERVAL_US) {
    lastSampleMicros = now;

    int raw = analogRead(EKG_INPUT);
    float voltage = (raw / 4095.0) * 3.3;

    float hp_filtered = applyFilter(voltage, hp_b, hp_a, hp_x, hp_y);
    float lp_filtered = applyFilter(hp_filtered, lp_b, lp_a, lp_x, lp_y);

    #ifdef ENABLE_WIFI_MQTT
    String payload = "{";
    payload += "\"device\":\"ECG-" + String(deviceNumber) + "\",";
    payload += "\"sample_ts\":" + String(micros() - startTimestamp) + ",";
    payload += "\"start_utc\":" + String((unsigned long)startUtcTimestamp) + ",";
    payload += "\"ecg\":" + String(lp_filtered, 3);
    payload += "}";

    client.publish(mqttTopic.c_str(), payload.c_str());
    #endif

    Serial.println(lp_filtered); // Debug-Output
}


    // Button Press → Deep Sleep
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Button pressed! Preparing for deep sleep...");
        enterDeepSleep();
    }

#ifdef ENABLE_WIFI_MQTT
    if (WiFi.status() != WL_CONNECTED && !connectToWiFi()) enterDeepSleep();
    if (!client.connected() && !connectToMQTT()) enterDeepSleep();
    client.loop();

    // Send status every 30 seconds
    if (ENABLE_STATUS_SENDING && (millis() - lastStatusSentMillis > statusIntervalMillis)) {
        lastStatusSentMillis = millis();

        float battery = getBatteryPercentage();
        bool padsConnectedNow = (digitalRead(LoPlus) == LOW && digitalRead(LoMinus) == LOW);

        String statusPayload = "{";
        statusPayload += "\"device\":\"ECG-" + String(deviceNumber) + "\",";
        statusPayload += "\"battery\":" + String(battery, 1) + ",";
        statusPayload += "\"pads_connected\":" + String(padsConnectedNow ? "true" : "false") + ",";
        statusPayload += "\"start_utc\":" + String((unsigned long)startUtcTimestamp);
        statusPayload += "}";

        client.publish((mqttTopic + "/status").c_str(), statusPayload.c_str());

        //Serial.println("Status message sent over MQTT.");
}


    // Resync NTP if needed
    if (millis() - lastNtpSyncMillis > ntpSyncInterval) {
        Serial.println("Resynchronizing NTP...");
        configTime(0, 0, "pool.ntp.org");
        lastNtpSyncMillis = millis();
    }
#endif


}

// ===========================
// Filter Function
// ===========================
float applyFilter(float input, float *b, float *a, float *x, float *y) {
    x[2] = x[1]; x[1] = x[0]; x[0] = input;
    y[2] = y[1]; y[1] = y[0];
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[1]*y[1] - a[2]*y[2];
    return y[0];
}

// ===========================
// Battery Monitoring
// ===========================
float getBatteryPercentage() {
    static float readings[BATTERY_SMOOTHING] = {0};
    static int idx = 0;
    static float sum = 0;

    int raw = analogRead(VBATPIN);
    float voltage = (raw / 4095.0) * 3.3 * VBAT_DIVIDER; 
    float percent = constrain(((voltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN)) * 100.0, 0, 100);

    sum -= readings[idx];
    readings[idx] = percent;
    sum += readings[idx];
    idx = (idx + 1) % BATTERY_SMOOTHING;

    return static_cast<int>(sum / BATTERY_SMOOTHING);
}


// ===========================
// Enter Deep Sleep
// ===========================
void enterDeepSleep() {
    Serial.flush();
    pinMode(LoPlus, INPUT_PULLUP);
    pinMode(LoMinus, INPUT_PULLUP);
    digitalWrite(Power_LED, LOW);
    digitalWrite(SDN_PIN, LOW);
    gpio_hold_en(GPIO_NUM_33);
    gpio_deep_sleep_hold_en();
    delay(1000);
    esp_deep_sleep_start();
}

// ===========================
// WiFi & MQTT Functions
// ===========================
#ifdef ENABLE_WIFI_MQTT
bool connectToWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        if (++wifiRetries >= MAX_RETRIES) return false;
    }
    Serial.println("\nWiFi connected!");
    Serial.println(WiFi.localIP());
    return true;
}

bool connectToMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        String clientId = "ECGClient-" + String(deviceNumber);
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected!");
            mqttRetries = 0;
            return true;
        }
        Serial.printf("Failed (rc=%d), retrying...\n", client.state());
        delay(5000);
        if (++mqttRetries >= MAX_RETRIES) return false;
    }
    return false;
}
#endif

// ===========================
// LED Flasher Task
// ===========================
void ledFlasherTask(void *pvParameters) {
    while (!setupComplete) {
        digitalWrite(Power_LED, HIGH);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        digitalWrite(Power_LED, LOW);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    digitalWrite(Power_LED, HIGH); // solid ON after setup
    vTaskDelete(NULL);
}
