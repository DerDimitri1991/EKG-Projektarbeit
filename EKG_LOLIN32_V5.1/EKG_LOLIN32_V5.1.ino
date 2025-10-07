// =============================
// Include Required Libraries
// =============================
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_sleep.h>
#include "driver/gpio.h"

// =====================
// Configuration Macros
// =====================
#define SAMPLE_RATE 250  // ECG sampling rate (Hz)
#define ENABLE_WIFI_MQTT // Comment this out to disable WiFi & MQTT features
#define deviceNumber 4 // define device Number
// ============================
// WiFi & MQTT Configuration
// ============================
#ifdef ENABLE_WIFI_MQTT
#define WIFI_SSID "DimaAlpha"
#define WIFI_PASSWORD "Qwerty**21***79"
//#define MQTT_BROKER "192.168.178.44" // Pi 2
#define MQTT_BROKER "192.168.178.104" // Pi 4
#define MQTT_PORT 1883
String mqttTopic = "ecg/device" + String(deviceNumber);
WiFiClient espClient;
PubSubClient client(espClient);
#endif

// ==================
// Global Variables
// ==================
Adafruit_ADS1115 ads;

#define CHUNK_SIZE 5
#define NUM_CHUNKS 500
#define BUFFER_SIZE (CHUNK_SIZE * NUM_CHUNKS)

unsigned long lastNtpSyncMillis = 0;
const unsigned long ntpSyncInterval = 10L * 60L * 1000L; // 10 Minutes in Milliseconds
time_t startUtcTimestamp = 0;

float ecgBuffer[BUFFER_SIZE];
unsigned long timeStamps[BUFFER_SIZE];
int bufferIndex = 0;
int messageId = 0;
unsigned long lastSampleMicros = 0;
unsigned long sampleIntervalMicros = 1000000 / SAMPLE_RATE;
unsigned long startTimestamp = 0;

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
#define SDN_PIN 33 //Pin to enable or Disable AD8232
#define Power_LED 32
#define VBATPIN 35

// Battery settings
#define VBAT_MIN 3.3
#define VBAT_MAX 4.2
#define BATTERY_SMOOTHING 10

// =====================
// Filter Coefficients
// =====================
// High-pass filter (0.5 Hz)
float hp_b[] = {0.995566, -1.991132, 0.995566};
float hp_a[] = {1.0, -1.991114, 0.991153};
float hp_x[3] = {0}, hp_y[3] = {0};

// Low-pass filter (40 Hz)
float lp_b[] = {0.206572, 0.413145, 0.206572};
float lp_a[] = {1.0, -0.369527, 0.195814};
float lp_x[3] = {0}, lp_y[3] = {0};

// ==========================
// MQTT Task & Queue Setup
// ==========================
TaskHandle_t mqttTaskHandle;
QueueHandle_t mqttQueue;

typedef struct {
    float ecg[CHUNK_SIZE];
    unsigned long ts[CHUNK_SIZE];
    int seq;
    int msgId;
    float battery;
    bool includeBattery;
    bool padsConnected;

} ECGMessage;

// ========================
// Setup Function
// ========================
void setup() {
    Serial.begin(115200);

    // Fix: Configure internal ADC to avoid Vref error
    analogReadResolution(12);              // 12-bit resolution (0–4095)
    analogSetAttenuation(ADC_11db);        // 0–3.3V range

    pinMode(Power_LED, OUTPUT);
    pinMode(LoPlus, INPUT);
    pinMode(LoMinus, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(SDN_PIN, OUTPUT);

    xTaskCreatePinnedToCore(ledFlasherTask, "LEDFLASH", 1000, NULL, 1, NULL, 1); // LED blinking until setup complete

    gpio_hold_dis(GPIO_NUM_33); // Unhold SDN after sleep

    Wire.begin(26, 27);
    Wire.setClock(50000);
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115.");
        while (1);
    }
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(SAMPLE_RATE);
    Serial.println("ADS1115 initialized successfully.");

    // Determine wake-up source
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke up from deep sleep.");
        digitalWrite(Power_LED, HIGH);
        digitalWrite(SDN_PIN, HIGH);
    } else {
        Serial.println("Powering on normally.");
        digitalWrite(Power_LED, LOW);
        digitalWrite(SDN_PIN, LOW);
    }

    // Enable deep sleep wake-up on button
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(BUTTON_PIN), 0);

#ifdef ENABLE_WIFI_MQTT
    if (!connectToWiFi()) enterDeepSleep();
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setBufferSize(2048);
    if (!connectToMQTT()) enterDeepSleep();

    configTime(0, 0, "pool.ntp.org"); // UTC-Time, no Timezone
    Serial.println("Warte auf NTP-Zeit...");
    time_t now = time(nullptr);
    while (now < 100000) { // Wait for Time
        delay(100);
        now = time(nullptr);
    }
    Serial.println("NTP-Zeit empfangen.");
    lastNtpSyncMillis = millis(); //Remember start time

    // Create MQTT queue and task
    mqttQueue = xQueueCreate(20, sizeof(ECGMessage));
    xTaskCreatePinnedToCore(mqttSendTask, "MQTTSendTask", 5000, NULL, 1, &mqttTaskHandle, 0);
#endif

    setupComplete = true;
    delay(100);                      // Give time for LED task to exit
    digitalWrite(Power_LED, HIGH);   // Make sure LED stays on
}



// ===========================================
// Main Loop - Sample, Filter, Chunk & Send
// ===========================================
void loop() {
    bool padsConnected = (digitalRead(LoPlus) == LOW && digitalRead(LoMinus) == LOW); //Leads Off detection

    unsigned long now = micros();
    //digitalWrite(Power_LED, HIGH); // Show system is running
    digitalWrite(SDN_PIN, HIGH);

    // ECG Sampling
    if (now - lastSampleMicros >= sampleIntervalMicros) {
        lastSampleMicros = now;
        if (bufferIndex == 0) {
          startTimestamp = now;
          startUtcTimestamp = time(nullptr);  // Remember UTC-Time 
      }


        timeStamps[bufferIndex] = now - startTimestamp;
        int16_t adcReading = ads.readADC_SingleEnded(0);
        float voltage = adcReading * 0.125;

        // Apply filters
        float hp_filtered = applyFilter(voltage, hp_b, hp_a, hp_x, hp_y);
        float lp_filtered = applyFilter(hp_filtered, lp_b, lp_a, lp_x, lp_y);

        ecgBuffer[bufferIndex] = lp_filtered;
        bufferIndex++;

        float battery = getBatteryPercentage();
        // Send if buffer full
        if (bufferIndex >= BUFFER_SIZE) {
            for (int chunk = 0; chunk < NUM_CHUNKS; chunk++) {
                ECGMessage msg;
                msg.seq = chunk;
                msg.msgId = messageId;
                msg.battery = battery;
                msg.includeBattery = (chunk == 0);
                msg.padsConnected = padsConnected;


                for (int i = 0; i < CHUNK_SIZE; i++) {
                    int idx = chunk * CHUNK_SIZE + i;
                    msg.ecg[i] = ecgBuffer[idx];
                    msg.ts[i] = timeStamps[idx];
                }
                xQueueSend(mqttQueue, &msg, portMAX_DELAY);
            }
            bufferIndex = 0;
            messageId++;
        }

        Serial.println(lp_filtered); // Optional debug output
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
    if (millis() - lastNtpSyncMillis > ntpSyncInterval) {
    Serial.println("Resynchronizing NTP...");
    configTime(0, 0, "pool.ntp.org");
    lastNtpSyncMillis = millis();
}

#endif


}

// ===========================
// Filter Function (IIR 2nd Order)
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
    float voltage = (raw / 1023.0) * 3.3 * 2.0; // 12-bit ADC, scale by 2 due to divider
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
// MQTT Send Task
// ===========================
#ifdef ENABLE_WIFI_MQTT
void mqttSendTask(void *parameter) {
    ECGMessage msg;
    while (true) {
        if (xQueueReceive(mqttQueue, &msg, portMAX_DELAY)) {
            String payload = "{";
            payload += "\"device\":\"ECG-" + String(deviceNumber) + "\",";
            payload += "\"msg_id\":" + String(msg.msgId);
            payload += ",\"pads_connected\":" + String(msg.padsConnected ? "true" : "false");
            payload += ",\"seq\":" + String(msg.seq);
            if (msg.includeBattery)
                payload += ",\"battery\":" + String(msg.battery, 1);

            // Append start_utc Only on the first Message
            if (msg.seq == 0) {
                payload += ",\"start_utc\":" + String((unsigned long)startUtcTimestamp);
            }

            payload += ",\"ecg\":[";
            for (int i = 0; i < CHUNK_SIZE; i++) {
                payload += String(msg.ecg[i], 3);
                if (i < CHUNK_SIZE - 1) payload += ",";
            }
            payload += "],\"ts\":[";
            for (int i = 0; i < CHUNK_SIZE; i++) {
                payload += String(msg.ts[i]);
                if (i < CHUNK_SIZE - 1) payload += ",";
            }
            payload += "]}";
            client.publish(mqttTopic.c_str(), payload.c_str());
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}
#endif

// ===========================
// LED Flasher Task (during setup)
// ===========================
void ledFlasherTask(void *pvParameters) {
    while (!setupComplete) {
        digitalWrite(Power_LED, HIGH);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        digitalWrite(Power_LED, LOW);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    digitalWrite(Power_LED, HIGH); // solid ON when setup is complete
    vTaskDelete(NULL);
}
