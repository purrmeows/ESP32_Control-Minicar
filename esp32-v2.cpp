#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ----- CONFIG -----
#define WIFI_STA_SSID   "btm02_2.4G"
#define WIFI_STA_PASS   "136167118"
#define WIFI_AP_SSID    "ESP32_CarAP"
#define WIFI_AP_PASS    "guthib123"  
//#define WIFI_AP_PASS    ""


#define MQTT_SERVER_IP  "192.168.2.33"
#define MQTT_PORT       1883
#define MQTT_TOPIC_CMD  "car/cmd"

#define DHTPIN          15
#define DHTTYPE         DHT11

#define TRIG_PIN        27
#define ECHO_PIN        34
#define OBSTACLE_CM     10

// Motor Pins (front axle + rear axle)
#define FRONT_PWM_PIN     18
#define FRONT_DIR_PIN1    32
#define FRONT_DIR_PIN2    33
#define FRONT_PWM_CH      0

#define SERVO_PIN         13

#define REAR_PWM_PIN      19
#define REAR_DIR_PIN1     25
#define REAR_DIR_PIN2     26
#define REAR_PWM_CH       1

#define LED_BLUE_PIN    2
#define LED_RED_PIN     4
#define BUTTON_INT_PIN  14

#define OLED_SDA        21
#define OLED_SCL        22
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1

// ----- GLOBALS -----
DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial BT;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Servo steering;

volatile bool emergencyInterrupt = false;
volatile unsigned long lastInterruptMillis = 0;
String latestCmd = "";
int requestedSpeed = 150;
String motionState = "Stopped";
float lastTemp=0, lastHum=0, lastDistance=999;

SemaphoreHandle_t cmdMutex;

// ----- PROTOTYPES -----
void setupWiFi();
void mqttCallback(char* topic, byte* message, unsigned int length);
void commsTask(void *pvParameters);
void sensorsTask(void *pvParameters);
void controlTask(void *pvParameters);
void stopMotors();
void setMotorAxle(int pwmVal,int dir,int channel,int dir1,int dir2);
void setAxles(int speedFront,int dirFront,int speedRear,int dirRear);
float readUltrasonicCM();
void IRAM_ATTR handleButtonInterrupt();
void showOLED();
void processCommand(String cmd);

// ----- SETUP -----
void setup() {
  Serial.begin(115200);

  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  ledcSetup(FRONT_PWM_CH, 20000, 8); ledcAttachPin(FRONT_PWM_PIN, FRONT_PWM_CH); ledcWrite(FRONT_PWM_CH,0);
  ledcSetup(REAR_PWM_CH,  20000, 8); ledcAttachPin(REAR_PWM_PIN, REAR_PWM_CH);   ledcWrite(REAR_PWM_CH,0);

  pinMode(FRONT_DIR_PIN1, OUTPUT); pinMode(FRONT_DIR_PIN2, OUTPUT);
  pinMode(REAR_DIR_PIN1, OUTPUT); pinMode(REAR_DIR_PIN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleButtonInterrupt, FALLING);

  dht.begin();
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  steering.attach(SERVO_PIN);
  steering.write(90);

  if(!BT.begin("ESP32_Car")) Serial.println("BT failed");

  setupWiFi();
  mqttClient.setServer(MQTT_SERVER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  cmdMutex = xSemaphoreCreateMutex();

  xTaskCreate(commsTask, "commsTask", 4096, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "sensorsTask", 4096, NULL, 1, NULL);
  xTaskCreate(controlTask, "controlTask", 4096, NULL, 2, NULL);
}

// ----- TASKS -----
void commsTask(void *pvParameters) {
  for(;;){
    // WiFi reconnect
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("WiFi disconnected! Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    }

    bool wifiConnected = WiFi.status()==WL_CONNECTED;
    digitalWrite(LED_BLUE_PIN, wifiConnected ? HIGH : LOW);

    // MQTT reconnect
    if(wifiConnected){
      if(!mqttClient.connected()){
        Serial.println("MQTT reconnecting...");
        if(mqttClient.connect("ESP32_Car")){
          mqttClient.subscribe(MQTT_TOPIC_CMD, 1);
          Serial.println("MQTT connected!");
        }
      } else mqttClient.loop();
    }

    // Bluetooth
    if(BT.available()){
      String cmd = BT.readStringUntil('\n');
      cmd.trim();
      if(cmd.length() > 0){
        if(xSemaphoreTake(cmdMutex, (TickType_t)10/portTICK_PERIOD_MS)==pdTRUE){
          latestCmd = cmd;
          xSemaphoreGive(cmdMutex);
          //Serial.printf("BT cmd=%s\n", cmd.c_str());
        }
      }
    }

    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

void sensorsTask(void *pvParameters){
  for(;;){
    float t=dht.readTemperature(); if(!isnan(t)) lastTemp=t;
    float h=dht.readHumidity(); if(!isnan(h)) lastHum=h;
    lastDistance=readUltrasonicCM();
    if(lastDistance<=OBSTACLE_CM && lastDistance>0){
      stopMotors();
      motionState="Stopped(Obstacle)";
    }
    showOLED();
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void controlTask(void *pvParameters){
  for(;;){
    if(emergencyInterrupt){
      digitalWrite(LED_RED_PIN,HIGH);
      digitalWrite(LED_BLUE_PIN,LOW);
      stopMotors();
    } else digitalWrite(LED_RED_PIN,LOW);

    if(xSemaphoreTake(cmdMutex,(TickType_t)10/portTICK_PERIOD_MS)==pdTRUE){
      String cmd=latestCmd; latestCmd=""; xSemaphoreGive(cmdMutex);
      if(cmd.length()>0){
        processCommand(cmd);
        Serial.printf("BT cmd=%s -> motionState=%s\n", cmd.c_str(), motionState.c_str());
      }
    }

    if(!emergencyInterrupt){
      int speedFront = requestedSpeed;
      int speedRear  = requestedSpeed;
      int dir = 0;
      if(motionState=="Forward") dir=1;
      else if(motionState=="Backward") dir=2;
      else if(motionState=="Stopped") dir=0;

      if(motionState=="Left") steering.write(60);
      else if(motionState=="Right") steering.write(120);
      else steering.write(90);

      setAxles(speedFront, dir, speedRear, dir);
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

// ----- HELPERS -----
void processCommand(String cmd){
  cmd.trim();
  if(cmd=="w") motionState="Forward";
  else if(cmd=="s") motionState="Backward";
  else if(cmd=="a") motionState="Left";
  else if(cmd=="d") motionState="Right";
  else if(cmd=="x" || cmd=="stop") motionState="Stopped";
  else if(cmd=="I"){ emergencyInterrupt=!emergencyInterrupt; }
  else if(cmd.toInt()>0) requestedSpeed=constrain(cmd.toInt(),0,255);
}

void stopMotors(){ 
  setMotorAxle(0,0,FRONT_PWM_CH,FRONT_DIR_PIN1,FRONT_DIR_PIN2);
  setMotorAxle(0,0,REAR_PWM_CH,REAR_DIR_PIN1,REAR_DIR_PIN2);
}

void setAxles(int speedFront,int dirFront,int speedRear,int dirRear){
  setMotorAxle(speedFront, dirFront, FRONT_PWM_CH, FRONT_DIR_PIN1, FRONT_DIR_PIN2);
  setMotorAxle(speedRear,  dirRear,  REAR_PWM_CH,  REAR_DIR_PIN1,  REAR_DIR_PIN2);
}

void setMotorAxle(int pwmVal,int dir,int channel,int dir1,int dir2){
  pwmVal=constrain(pwmVal,0,255);
  if(dir==0){ digitalWrite(dir1,LOW); digitalWrite(dir2,LOW); ledcWrite(channel,0); }
  else if(dir==1){ digitalWrite(dir1,HIGH); digitalWrite(dir2,LOW); ledcWrite(channel,pwmVal); }
  else if(dir==2){ digitalWrite(dir1,LOW); digitalWrite(dir2,HIGH); ledcWrite(channel,pwmVal); }
}

float readUltrasonicCM(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN,LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if(dur == 0) return 999;
  return (dur / 2.0) / 29.1;
}

void IRAM_ATTR handleButtonInterrupt(){
  unsigned long now=millis();
  if(now-lastInterruptMillis<300) return;
  lastInterruptMillis=now;
  emergencyInterrupt=!emergencyInterrupt;
}

void showOLED(){
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0);
  display.printf("State: %s\n",motionState.c_str());
  display.printf("Temp: %.1fC Hum: %.1f%%\n",lastTemp,lastHum);
  display.printf("Dist: %.1f cm\n",lastDistance);
  display.printf("Speed: %d\n",requestedSpeed);
  display.display();
}

void mqttCallback(char* topic, byte* message, unsigned int length){
  String msg="";
  for(unsigned int i=0;i<length;i++) msg+=(char)message[i];
  msg.trim(); 
  if(msg.length()>0){
    processCommand(msg);
    Serial.printf("MQTT msg=%s -> motionState=%s\n", msg.c_str(), motionState.c_str());
  }
}

void setupWiFi(){
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);  
  //WiFi.softAP(WIFI_AP_SSID);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else Serial.println("\nFailed to connect WiFi");
}

void loop() {
  // FreeRTOS tasks handle main work
}
