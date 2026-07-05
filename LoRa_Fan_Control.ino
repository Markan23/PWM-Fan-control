// =====================================================
// ESP32-C3 SuperMini — Dual Fan + 3x DS18B20 + LoRa
// Compatible with your existing gateway
// =====================================================

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==================== PINS (SuperMini) ====================
#define ONE_WIRE_BUS  8       // DS18B20 Data

#define PWM1_PIN      9       // Fan 1 PWM
#define TACH1_PIN     20      // Fan 1 Tach

#define PWM2_PIN      0       // Fan 2 PWM
#define TACH2_PIN     1       // Fan 2 Tach

// LoRa
#define LORA_SS       10
#define LORA_RST      4
#define LORA_DIO0     3

#define MY_ADDRESS      30     // This node addr
#define GATEWAY_ADDRESS 254    // My MQTT_LoRa Gateway address
#define LORA_FREQ       868.1

const int PWM_FREQ = 25000;
const int PWM_RESOLUTION = 8;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

RH_RF95 rf95(LORA_SS, LORA_DIO0);
RHReliableDatagram manager(rf95, MY_ADDRESS);

volatile unsigned long tach1Count = 0;
volatile unsigned long tach2Count = 0;

float tempZone1 = 0.0;     // Near Fan 1
float tempZone2 = 0.0;     // Near Fan 2
float tempAmbient = 0.0;

int fan1Speed = 30;
int fan2Speed = 30;
bool autoMode1 = true;
bool autoMode2 = true;
float tempOn1 = 25.0;
float tempFull1 = 50.0;
float tempOn2 = 35.0;
float tempFull2 = 50.0;

unsigned long rpm1 = 0;
unsigned long rpm2 = 0;
unsigned long lastTachTime = 0;
unsigned long lastSendTime = 0;

void IRAM_ATTR tach1ISR() { tach1Count++; }
void IRAM_ATTR tach2ISR() { tach2Count++; }

void setup() {
  Serial.begin(115200);
  delay(2000);

  sensors.begin();
  sensors.setResolution(12);

  // PWM
  ledcAttach(PWM1_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM2_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM1_PIN, 0);
  ledcWrite(PWM2_PIN, 0);

  // Tachometers (safe until fans are connected)
  pinMode(TACH1_PIN, INPUT_PULLUP);
  pinMode(TACH2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH1_PIN), tach1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(TACH2_PIN), tach2ISR, FALLING);

  // LoRa
  SPI.begin(6, 2, 7, 10);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW); delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);

  if (manager.init()) {
    rf95.setFrequency(LORA_FREQ);
    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
    rf95.setTxPower(14, false);
    Serial.println("LoRa initialized successfully");
  } else {
    Serial.println("LoRa init failed!");
  }

  lastTachTime = lastSendTime = millis();
  Serial.println("SuperMini - 3 Temp Sensors + Dual Fan Ready");
}

void loop() {
  unsigned long now = millis();

  // Read all 3 DS18B20 sensors
  sensors.requestTemperatures();
  tempZone1   = sensors.getTempCByIndex(2);
  tempZone2   = sensors.getTempCByIndex(1);
  tempAmbient = sensors.getTempCByIndex(0);

  if (tempZone1 == DEVICE_DISCONNECTED_C) tempZone1 = -127.0;
  if (tempZone2 == DEVICE_DISCONNECTED_C) tempZone2 = -127.0;
  if (tempAmbient == DEVICE_DISCONNECTED_C) tempAmbient = -127.0;

  // Receive LoRa commands
  // if (now % 200 == 0) receiveLoRaCommands();
  receiveLoRaCommands();

  // Apply fan speeds
  ledcWrite(PWM1_PIN, map(getFanSpeed(1), 0, 100, 0, 255));
  ledcWrite(PWM2_PIN, map(getFanSpeed(2), 0, 100, 0, 255));

  // Periodic report
  if (now - lastSendTime >= 30000) {   // 2 minutes
    sendLoRaPacket();
    lastSendTime = now;
  }

  delay(10);   // Keep watchdog happy
}

// ====================== FUNCTIONS ======================
int getFanSpeed(int fanId) {
  float t = (fanId == 1) ? tempZone1 : tempZone2;
  if (fanId == 1) {
    if (!autoMode1) return fan1Speed;
    if (t < tempOn1) return 0;
    if (t >= tempFull1) return 100;
    return map(t*10, tempOn1*10, tempFull1*10, 30, 100);
  } else {
    if (!autoMode2) return fan2Speed;
    if (t < tempOn2) return 0;
    if (t >= tempFull2) return 100;
    return map(t*10, tempOn2*10, tempFull2*10, 30, 100);
  }
}

void receiveLoRaCommands() {
  if (!manager.available()) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAck(buf, &len, &from)) {
    buf[len] = 0;
    String msg = String((char*)buf);
    Serial.print("Cmd: "); Serial.println(msg);

    if (msg.indexOf("\"cmd\":\"mode\"") != -1) {
      int id = extractInt(msg, "\"id\":", 0);
      bool newAuto = msg.indexOf("\"auto\":true") != -1;
      if (id == 1) autoMode1 = newAuto;
      else if (id == 2) autoMode2 = newAuto;
      else { autoMode1 = newAuto; autoMode2 = newAuto; }
    }
    else if (msg.indexOf("\"speed\"") != -1) {
      int id = extractInt(msg, "\"id\":", 0);
      int speed = extractInt(msg, "\"speed\":", 30);
      speed = constrain(speed, 0, 100);
      if (id == 1) { fan1Speed = speed; autoMode1 = false; }
      else if (id == 2) { fan2Speed = speed; autoMode2 = false; }
      else { fan1Speed = speed; fan2Speed = speed; autoMode1 = false; autoMode2 = false; }
    }
    else if (msg.indexOf("\"thresh\"") != -1) {
      int id = extractInt(msg, "\"id\":", 0);
      float onVal = extractFloat(msg, "\"on\":", 35.0);
      float fullVal = extractFloat(msg, "\"full\":", 60.0);
      if (id == 1) { tempOn1 = onVal; tempFull1 = fullVal; }
      else if (id == 2) { tempOn2 = onVal; tempFull2 = fullVal; }
      else {
        tempOn1 = onVal; tempFull1 = fullVal;
        tempOn2 = onVal; tempFull2 = fullVal;
      }
    }
  }
}

void sendLoRaPacket() {
  char payload[200];
  snprintf(payload, sizeof(payload),
    "{\"T1\":%.2f,\"T2\":%.2f,\"TA\":%.2f,\"F1\":%d,\"A1\":%s,\"R1\":%lu,\"F2\":%d,\"A2\":%s,\"R2\":%lu}",
    tempZone1, tempZone2, tempAmbient,
    getFanSpeed(1), autoMode1 ? "true" : "false", rpm1,
    getFanSpeed(2), autoMode2 ? "true" : "false", rpm2);

  manager.sendtoWait((uint8_t*)payload, strlen(payload), GATEWAY_ADDRESS);
  Serial.print("Sent: ");
  Serial.println(payload);
}

// ====================== HELPERS ======================
int extractInt(String &s, const char* key, int def) {
  int start = s.indexOf(key) + strlen(key);
  if (start < strlen(key)) return def;
  String val = s.substring(start);
  int end = min(val.indexOf(','), val.indexOf('}'));
  if (end > 0) val = val.substring(0, end);
  return val.toInt();
}

float extractFloat(String &s, const char* key, float def) {
  int start = s.indexOf(key) + strlen(key);
  if (start < strlen(key)) return def;
  String val = s.substring(start);
  int end = min(val.indexOf(','), val.indexOf('}'));
  if (end > 0) val = val.substring(0, end);
  return val.toFloat();
}
