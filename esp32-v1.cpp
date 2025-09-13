#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>

#define WIFI_SSID   "btm02_2.4G"
#define WIFI_PASS   "136167118"
#define MQTT_SERVER "192.168.2.33"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "car/cmd"

BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

String latestCmd = "";

// MQTT callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT callback fired, topic: ");
  Serial.println(topic);
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  Serial.print("Received MQTT message: ");
  Serial.println(msg);  // เพิ่มการพิมพ์ข้อความที่รับจาก MQTT
  if (msg.length() > 0) {
    latestCmd = msg;
  }
}

void setup() {
  Serial.begin(115200);

  // เชื่อมต่อ WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // ตั้งค่าการเชื่อมต่อ MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // เริ่มต้น Bluetooth
  BT.begin("ESP32_Car");

  Serial.println("ESP32 is ready!");
}

void loop() {
  // ตรวจสอบการเชื่อมต่อ MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      Serial.println("MQTT not connected...");
      if (mqttClient.connect("ESP32_Car")) {
        Serial.println("MQTT connected!");
        mqttClient.subscribe(MQTT_TOPIC, 1);
      } else {
        Serial.print("MQTT connection failed, state: ");
        Serial.println(mqttClient.state());  // พิมพ์รหัสสถานะการเชื่อมต่อ
        delay(500);
      }
    } else {
      mqttClient.loop();
    }
  }

  // ตรวจสอบคำสั่ง Bluetooth
  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim();
    Serial.print("Received Bluetooth command: ");
    Serial.println(cmd);  // เพิ่มการพิมพ์คำสั่งที่ได้รับจาก Bluetooth
    if (cmd.length() > 0) {
      latestCmd = cmd;
    }
  }

  // ประมวลผลคำสั่งที่ได้รับ
  if (latestCmd == "w") {
    Serial.println("Move Forward");
    // ที่นี่ใส่โค้ดขับมอเตอร์ forward
    latestCmd = "";
  }

  if (latestCmd == "a") {
    Serial.println("Move left");
    // ที่นี่ใส่โค้ดขับมอเตอร์ left
    latestCmd = "";
  }

  if (latestCmd == "s") {
    Serial.println("Move backward");
    // ที่นี่ใส่โค้ดขับมอเตอร์ backward
    latestCmd = "";
  }

  if (latestCmd == "d") {
    Serial.println("Move right");
    // ที่นี่ใส่โค้ดขับมอเตอร์ right
    latestCmd = "";
  }

  delay(100);
}
