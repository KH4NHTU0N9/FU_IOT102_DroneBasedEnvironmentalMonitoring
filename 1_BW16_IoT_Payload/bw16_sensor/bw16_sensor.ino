// bw16_sensor.ino — BW16 Drone IoT Payload
// DHT22 + MQ-135 + Servo + OLED → MQTT

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

// --- Network Config ---
const char* ssid        = SECRET_SSID;
const char* password    = SECRET_PASS;
const char* mqtt_server = "broker.hivemq.com";
const char* topic_sensors  = "iot102_drone/payload/sensors";
const char* topic_payload  = "iot102_drone/control/payload";
const int   mqtt_port   = 1883;

// --- Pin Definitions ---
#define DHT_PIN         PA30
#define DHT_TYPE        DHT22
#define MQ135_PIN       PB3


#define BUZZER_PIN      PA14
#define LED_PIN         PA15
#define LED_GREEN_PIN   PA27

// --- OLED Config ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Active Levels ---
#define BUZZER_ACTIVE   LOW
#define LED_ACTIVE      HIGH

#define BUZZER_ON       BUZZER_ACTIVE
#define BUZZER_OFF      (!BUZZER_ACTIVE)
#define LED_ON          LED_ACTIVE
#define LED_OFF         (!LED_ACTIVE)

// --- Thresholds & Constants ---
const int   CO2_THRESHOLD      = 600;
const float TEMP_THRESHOLD     = 35.0;
const int   WIFI_MAX_RETRIES   = 30;
const int   MQTT_MAX_RETRIES   = 5;
const int   MQTT_RETRY_DELAY   = 3000;
const int   CUSTOM_MQTT_KEEPALIVE = 60;
const unsigned long MQ135_WARMUP_MS  = 30000;
const uint8_t OLED_I2C_ADDR    = 0x3C;

// --- MQTT Override State ---
bool mqtt_buzzer_override = false;
bool mqtt_buzzer_state    = false;
bool mqtt_led_override    = false;
bool mqtt_led_state       = false;

// --- Objects ---
WiFiClient   wifiClient;
PubSubClient client(wifiClient);
DHT          dht(DHT_PIN, DHT_TYPE);
unsigned long lastMsg = 0;
unsigned long lastOLEDUpdate = 0;
const long    interval = 2000;
const long    oledInterval = 200;

// Sensor globals
float temp_val = 0.0;
float hum_val  = 0.0;
int   mq_raw_val = 0;

bool  env_alert = false;

// [FIX] Cache RSSI để tránh WiFi.RSSI() block 50ms mỗi 200ms → phá PWM servo
int   cached_rssi = 0;
unsigned long lastRssiUpdate = 0;
const long    rssiInterval = 5000;  // Cập nhật RSSI mỗi 5 giây

// ============================================================
//  JSON Parser
// ============================================================
String parseJsonField(const String& json, const String& key) {
    int keyIndex = json.indexOf("\"" + key + "\"");
    if (keyIndex == -1) return "";
    int colonIndex = json.indexOf(":", keyIndex);
    if (colonIndex == -1) return "";

    int valStartIndex = colonIndex + 1;
    while (valStartIndex < json.length() && (json[valStartIndex] == ' ' || json[valStartIndex] == '\t' || json[valStartIndex] == '\r' || json[valStartIndex] == '\n')) {
        valStartIndex++;
    }
    if (valStartIndex >= json.length()) return "";

    if (json[valStartIndex] == '"') {
        int valEndIndex = json.indexOf("\"", valStartIndex + 1);
        if (valEndIndex == -1) return "";
        return json.substring(valStartIndex + 1, valEndIndex);
    } else {
        int commaIndex = json.indexOf(",", valStartIndex);
        int braceIndex = json.indexOf("}", valStartIndex);
        int valEndIndex;
        if (commaIndex == -1) valEndIndex = braceIndex;
        else if (braceIndex == -1) valEndIndex = commaIndex;
        else valEndIndex = min(commaIndex, braceIndex);

        if (valEndIndex == -1) valEndIndex = json.length();
        String val = json.substring(valStartIndex, valEndIndex);
        val.trim();
        return val;
    }
}

// ============================================================
//  MQTT Callback
// ============================================================
void callback(char* topic, byte* payload, unsigned int length) {
    String msgString;
    msgString.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
        msgString += (char)payload[i];
    }

    Serial.print("[MQTT] topic=");
    Serial.print(topic);
    Serial.print(" msg=");
    Serial.println(msgString);

    // ── Route theo topic ──────────────────────────────────────
    String topicStr(topic);



    // 2. Command từ Web Dashboard (trên topic_payload)
    String command = parseJsonField(msgString, "command");

    if (command == "BUZZER_ON") {
        mqtt_buzzer_override = true;
        mqtt_buzzer_state    = true;
        digitalWrite(BUZZER_PIN, BUZZER_ON);
        Serial.println("[CMD] BUZZER ON");
    } else if (command == "BUZZER_OFF") {
        mqtt_buzzer_override = true;
        mqtt_buzzer_state    = false;
        digitalWrite(BUZZER_PIN, BUZZER_OFF);
        Serial.println("[CMD] BUZZER OFF");
    } else if (command == "LED_ON") {
        mqtt_led_override = true;
        mqtt_led_state    = true;
        digitalWrite(LED_PIN, LED_ON);
        digitalWrite(LED_GREEN_PIN, LED_OFF);
        Serial.println("[CMD] LED RED ON");
    } else if (command == "LED_OFF") {
        mqtt_led_override = true;
        mqtt_led_state    = false;
        digitalWrite(LED_PIN, LED_OFF);
        digitalWrite(LED_GREEN_PIN, LED_ON);
        Serial.println("[CMD] LED GREEN ON");
    } else if (command == "DISARM") {
        Serial.println("[CMD] DISARM (no local action)");
    } else if (command == "RESET") {
        mqtt_buzzer_override = false;
        mqtt_led_override    = false;
        Serial.println("[CMD] RESET auto mode");
    }
}

// ============================================================
//  WiFi Connect
// ============================================================
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;

    Serial.print("[WiFi] Connecting to: ");
    Serial.println(ssid);
    WiFi.begin(const_cast<char*>(ssid), const_cast<char*>(password));

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < WIFI_MAX_RETRIES) {
        delay(500);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[WiFi] OK! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("[WiFi] Failed. Will retry later.");
    }
}

// ============================================================
//  MQTT Connect
// ============================================================
void connectMQTT() {
    int retry = 0;
    while (!client.connected() && retry < MQTT_MAX_RETRIES) {
        Serial.print("[MQTT] Connecting...");

        String clientId = "DroneIoT_BW16_" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println(" OK!");
            client.subscribe(topic_payload);   // lệnh từ Web
            break;
        } else {
            Serial.print(" Failed, rc=");
            Serial.print(client.state());
            Serial.println(" retry in 3s...");
            delay(MQTT_RETRY_DELAY);
        }
        retry++;
    }
}

// ============================================================
//  OLED Display
// ============================================================
void updateOLED(bool env_alert) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // ═══════════════════════════════════════════════════
    // VÙNG VÀNG (y = 0–15): Tiêu đề / Trạng thái cảnh báo
    // ═══════════════════════════════════════════════════
    display.setTextSize(1);
    display.setCursor(0, 3);
    if (env_alert) {
        if (temp_val >= TEMP_THRESHOLD && mq_raw_val > CO2_THRESHOLD) {
            display.print("** NGUY HIEM KEP! **");
        } else if (temp_val >= TEMP_THRESHOLD) {
            display.print("** CANH BAO NHIET! **");
        } else {
            display.print("** CANH BAO O NHIEM **");
        }
    } else {
        display.print("* DRONE IOT & TELEM *");
    }

    // ═══════════════════════════════════════════════════
    // VÙNG XANH (y = 16–63): Kết hợp Cảm biến
    // ═══════════════════════════════════════════════════

    // Dòng 1 (y = 16): Nhiệt độ
    display.setCursor(0, 16);
    display.print("Temp: ");
    if (temp_val == 0.0 && hum_val == 0.0) {
        display.print("--.-");
    } else {
        display.print(temp_val, 1);
    }
    display.print(" C");
    if (temp_val >= TEMP_THRESHOLD) display.print("!");

    // Dòng 2 (y = 28): Độ ẩm
    display.setCursor(0, 28);
    display.print("Hum : ");
    if (hum_val == 0.0 && temp_val == 0.0) {
        display.print("--");
    } else {
        display.print(hum_val, 0);
    }
    display.print(" %");

    // Dòng 3 (y = 40): Khí Gas / CO2
    display.setCursor(0, 40);
    display.print("Gas : ");
    display.print(mq_raw_val);
    display.print(" PPM");
    if (env_alert) {
        display.print(" (!)");
    }

    // Dòng 4 (y = 52): Sóng WiFi
    display.setCursor(0, 52);
    display.print("WiFi RSSI: ");
    display.print(cached_rssi);
    display.print(" dBm");

    display.display();
}

// ============================================================
//  Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==============================");
    Serial.println("  BW16 Drone IoT Payload");
    Serial.println("==============================");

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_GREEN_PIN, LED_ON);

    pinMode(MQ135_PIN, INPUT);


    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println("[ERROR] OLED init failed");
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,10);
        display.println("Drone IoT Booting...");
        display.display();
    }

    dht.begin();
    delay(500);

    connectWiFi();

    wifiClient.setBlockingMode();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setKeepAlive(CUSTOM_MQTT_KEEPALIVE);

    Serial.println("[INIT] Setup complete!");
}

// ============================================================
//  Main Loop
// ============================================================
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
        connectMQTT();
    }

    client.loop();

    unsigned long now = millis();

    if (now - lastMsg >= interval) {
        lastMsg = now;

        float t_new = dht.readTemperature();
        float h_new = dht.readHumidity();

        bool dht_ok = true;
        if (isnan(t_new) || isnan(h_new)) {
            // [FIX] Giữ nguyên giá trị cũ thay vì về 0
            // → Tránh OLED và MQTT nhảy 0 xen kẽ
            dht_ok = false;
            Serial.println("[DHT] Read failed, keeping last value");
        } else {
            temp_val = t_new;
            hum_val  = h_new;
        }

        mq_raw_val = analogRead(MQ135_PIN);



        env_alert = (temp_val >= TEMP_THRESHOLD);
        if (now > MQ135_WARMUP_MS) {
            env_alert = env_alert || (mq_raw_val > CO2_THRESHOLD);
        }

        if (mqtt_buzzer_override) {
            digitalWrite(BUZZER_PIN, mqtt_buzzer_state ? BUZZER_ON : BUZZER_OFF);
        } else {
            digitalWrite(BUZZER_PIN, env_alert ? BUZZER_ON : BUZZER_OFF);
        }

        if (mqtt_led_override) {
            digitalWrite(LED_PIN,       mqtt_led_state ? LED_ON : LED_OFF);
            digitalWrite(LED_GREEN_PIN, mqtt_led_state ? LED_OFF : LED_ON);
        } else {
            digitalWrite(LED_PIN,       env_alert ? LED_ON : LED_OFF);
            digitalWrite(LED_GREEN_PIN, env_alert ? LED_OFF : LED_ON);
        }

        char jsonBuf[256];
        snprintf(jsonBuf, sizeof(jsonBuf),
            "{\"temp\":%.1f,\"humidity\":%.1f,\"co2\":%d,\"alert\":%d,\"rssi\":%d,\"dht_ok\":%d}",
            temp_val, hum_val, mq_raw_val, env_alert ? 1 : 0, (int)WiFi.RSSI(), dht_ok ? 1 : 0);

        if (client.connected() && client.publish(topic_sensors, jsonBuf)) {
            Serial.print("[SEND] ");
            Serial.println(jsonBuf);
        } else {
            Serial.println("[SEND] MQTT publish failed.");
        }
    }

    if (now - lastOLEDUpdate >= oledInterval) {
        lastOLEDUpdate = now;
        updateOLED(env_alert);
    }


    // [FIX] Cập nhật RSSI riêng mỗi 5s để không block PWM servo
    if (now - lastRssiUpdate >= rssiInterval) {
        lastRssiUpdate = now;
        if (WiFi.status() == WL_CONNECTED) {
            cached_rssi = (int)WiFi.RSSI();
        }
    }
}
