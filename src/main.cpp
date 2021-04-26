#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "MPU9250.h"
#include <WiFiManager.h>
#include "config.h"

#define SDA 21
#define SCL 22

const int ledPin1 = 17;
const int ledPin2 = 18;

const int PERIOD_FIVE_MINUTES = 300000;
const int PERIOD_TEN_SECONDS = 10000;
const int PERIOD_ONE_SECOND = 1000;
const int PERIOD_FIVE_SECOND = 5000;
const int PERIOD_TWO_MS = 2;
const int BUFFER_SIZE = 200;
const int BUFFER_SIZE_TEN_SEC = 20;
const int BUFFER_SIZE_FIVE_MIN = 50;
const int BAUD_RATE = 115200;

char *data_endpoint = "iot-vib/data";
char *broadcast_endpoint = "iot-vib/broadcast";

int currentStateFiveM = 0; //assume machine machineStateFiveM is ON initially
int lastStateFiveM;
int offCount = 0;
int trueOffCount = 0;
char *machineState = "TRUEOFF";
int currentStateTenS = 0;
int lastStateTenS;

String stateOffset;
String stateFiveMThreshold;
String stateTenSThreshold;

MPU9250 mpu;
String mqttClientId;
WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long current_time = 0.0;
unsigned long elapsed_time_ten;
unsigned long elapsed_time_one;
unsigned long elapsed_time_one_mac;
unsigned long elapsed_time_five;
unsigned long elapsed_time_two_ms;
unsigned long elapsed_time_fivemins;

int buff_counter = 0;
float buff[BUFFER_SIZE];
bool wire_status = false;
int buff_ten_sec_counter = 0;
int buff_five_min_counter = 0;
bool is_mpu_available = false;
bool is_client_connected = false;
bool is_mac_verified = false;
bool is_state_given = false;
StaticJsonDocument<1000> JSONDocument;
float buff_ten_sec[BUFFER_SIZE_TEN_SEC];
float buff_five_min[BUFFER_SIZE_FIVE_MIN];

void led_init();
void setup_wifi();
void setup_mqtt();
void sample_data();
void updateMachineState();
void espclient_reconnect();
void aggregate_one_second_data();
void aggregate_ten_second_data();
void computeMachineState5m();
void executeAfter(int, unsigned long, void (*callback)(void));
void handleMqttRequestResponse(char *, byte *, unsigned int);
char *get_data_endpoint();
bool sendMessage(JsonObject, char *);
JsonObject prepareDataPayload(double, double, double, int, double, double, String);
JsonObject prepareBroadcastPayload();
JsonObject prepareNotificationPayload();
JsonObject prepareBroadcastStatePayload();

void led_init()
{
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
}

void setup_wifi()
{
  WiFiManager wm;
  // wm.resetSettings();
  bool res;
  res = wm.autoConnect("LNS_ESP", "lns@P@ssw0rd"); // password protected ap
  if (!res)
  {
    Serial.println("Failed to connect");
  }
  else
  {
    Serial.println("Connection Successful!");
  }
}

void executeAfter(int threshold_time, unsigned long *previous_time, void (*callback)(void))
{
  if (current_time - *previous_time >= threshold_time)
  {
    callback();
    *previous_time = current_time;
  }
}

int get_sample_data_len()
{
  return sizeof buff / sizeof *buff;
}

void broadcast_mac_address()
{
  sendMessage(prepareBroadcastPayload(), "iot-vib/broadcast");
}

void broadcast_state_query()
{
  sendMessage(prepareBroadcastStatePayload(), "iot-vib/broadcast/state");
}

void setup_mqtt()
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(handleMqttRequestResponse);
}

void handleMqttRequestResponse(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/output")
  {
    Serial.print("Changing output to ");
    Serial.println(messageTemp);
  }

  if (String(topic) == "iot-vib/calibrate")
  {
    Serial.print("message received ");
    Serial.println(messageTemp);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, messageTemp);
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    String deviceMacId = doc["deviceMACId"];
    if (deviceMacId == WiFi.macAddress())
    {
      mpu.calibrateAccelGyro();
      delay(3000);
      ESP.restart();
    }
  }
  if (String(topic) == "iot-vib/broadcast/verify")
  {
    Serial.print("message received ");
    Serial.println(messageTemp);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, messageTemp);
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    String deviceMacId = doc["deviceMACId"];
    bool verified = doc["verified"];
    if (deviceMacId == WiFi.macAddress() && verified)
    {
      is_mac_verified = true;
    }
    else if (deviceMacId == WiFi.macAddress() && !verified)
    {
      is_mac_verified = false;
    }
  }

  if (String(topic) == "iot-vib/broadcast/state/verify" && is_mac_verified)
  {
    Serial.print("Threshold is:");
    Serial.println(messageTemp);
    StaticJsonDocument<300> state;
    DeserializationError err = deserializeJson(state, messageTemp);
    if (err)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(err.f_str());
      return;
    }
    String deviceMacIdState = state["deviceMACId"];
    String offset = state["offset"];
    String fiveMThreshold = state["fiveMThreshold"];
    String tenSThreshold = state["tenSThreshold"];
    bool stateVerified = state["stateVerified"];
    if (deviceMacIdState != WiFi.macAddress())
    {
      return;
    }
    stateOffset = offset;
    stateFiveMThreshold = fiveMThreshold;
    stateTenSThreshold = tenSThreshold;
    if (deviceMacIdState == WiFi.macAddress() && stateVerified)
    {
      is_state_given = true;
    }
    else
    {
      is_state_given = false;
    }
  }

  if (String(topic) == "iot-vib/device-restart" && is_mac_verified)
  {
    Serial.print("Delete Data:");
    Serial.println(messageTemp);
    StaticJsonDocument<300> doc;
    DeserializationError err = deserializeJson(doc, messageTemp);
    if (err)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(err.f_str());
      return;
    }
    String deviceMacId = doc["deviceMACId"];
    if (deviceMacId == WiFi.macAddress())
    {
      ESP.restart();
    }
  }
}

void sample_data()
{
  is_mpu_available = mpu.update();
  if (is_mpu_available && buff_counter <= BUFFER_SIZE)
  {
    double total = sqrt(pow(mpu.getAccX() * 100, 2) + pow(mpu.getAccY() * 100, 2) + pow(mpu.getAccZ() * 0.1, 2));
    buff[buff_counter] = total;
    buff_counter++;
  }
}

void aggregate_one_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_counter; i++)
  {
    mean += buff[i];
  }
  mean = (mean / buff_counter);
  buff_counter = 0;
  if (buff_ten_sec_counter <= BUFFER_SIZE_TEN_SEC)
  {
    buff_ten_sec[buff_ten_sec_counter] = mean;
    buff_ten_sec_counter++;
  }
  Serial.print("One second mean data = ");
  Serial.println(mean);
}

void aggregate_ten_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_ten_sec_counter; i++)
  {
    mean += buff_ten_sec[i];
  }
  mean = (mean / buff_ten_sec_counter);
  if (buff_five_min_counter <= BUFFER_SIZE_FIVE_MIN)
  {
    buff_five_min[buff_five_min_counter] = mean;
    buff_five_min_counter++;
  }
  Serial.print("10 second Mean data ");
  Serial.println(mean);
  buff_ten_sec_counter = 0;
  // sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 10, mean), "iot-vib/data");
  lastStateTenS = currentStateTenS;
  double comparedMean = 0.0;
  comparedMean = isnan(mean) ? 0.0 : abs(mean - stateOffset.toDouble()) < stateTenSThreshold.toDouble();

  if (isnan(mean) || abs(mean - stateOffset.toDouble()) < stateTenSThreshold.toDouble())
  {
    currentStateTenS = 0;
  }
  else
  {
    currentStateTenS = 1;
  }
  updateMachineState();
  if (machineState == "OFF")
  {
    offCount++;
  }

  // Device is off for 4 minutes
  if (offCount >= 24) // incrementing in 10 second loop so
  {
    trueOffCount++;
    // Device is off for 8 minutes
    if (trueOffCount >= 2)
    {
      machineState = "TRUEOFF";
    }
    if (trueOffCount == 2)
    {
      sendMessage(prepareNotificationPayload(), "iot-vib/notification");
    }
  }

  sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 10, mean, comparedMean, machineState), "iot-vib/data");
}

void updateMachineState()
{
  if (lastStateTenS || currentStateTenS)
  {
    machineState = "ON";
    offCount = 0;
    trueOffCount = 0;
  }
  else
  {
    if (machineState != "TRUEOFF")
    {
      machineState = "OFF";
    }
  }
}

void espclient_reconnect()
{

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientId.c_str()))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      client.subscribe("iot-vib/broadcast/verify");
      client.subscribe("iot-vib/broadcast/state/verify");
      client.subscribe("iot-vib/device-restart");
      is_client_connected = true;
    }
    else
    {
      is_client_connected = false;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

JsonObject prepareDataPayload(double ax, double ay, double az, int period, double mean, double comparedMean, String deviceState)
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["macId"] = WiFi.macAddress();
  jsonObject["ds"] = deviceState;
  jsonObject["ax"] = ax;
  jsonObject["ay"] = ay;
  jsonObject["az"] = az;
  jsonObject["mean"] = mean;
  jsonObject["cm"] = comparedMean;
  jsonObject["p"] = period;
  jsonObject["tenSTh"] = stateTenSThreshold;
  jsonObject["fiveMTh"] = stateFiveMThreshold;
  jsonObject["ofst"] = stateOffset;

  return jsonObject;
}

JsonObject prepareBroadcastPayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["isVerified"] = is_mac_verified;
  return jsonObject;
}

JsonObject prepareNotificationPayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["sendNoti"] = true;
  return jsonObject;
}

JsonObject prepareBroadcastStatePayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["isStateGiven"] = is_state_given;
  return jsonObject;
}

bool sendMessage(JsonObject jsonObject, char *endpoint)
{
  char JSONmessageBuffer[200];
  serializeJson(jsonObject, JSONmessageBuffer);
  if (client.publish(endpoint, JSONmessageBuffer) == true)
  {
    return true;
  }
  else
  {
    Serial.println("Error sending message");
    return false;
  }
}

void setup()
{
  WiFi.mode(WIFI_STA);
  Serial.begin(BAUD_RATE);
  wire_status = Wire.begin();
  while (!Serial || !wire_status)
  {
  }
  led_init();
  setup_wifi();
  String clientId = "ESP32-";
  mqttClientId = clientId + String(random(0xffff), HEX);
  setup_mqtt();
  current_time = millis();
  mpu.setup(0x68);
  delay(5000);
}

void loop()
{
  if (!client.connected())
  {
    espclient_reconnect();
  }
  client.loop();
  if (is_client_connected)
  {
    digitalWrite(ledPin1, HIGH);
    current_time = millis();
    if (!is_mac_verified)
    {
      executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one_mac, &broadcast_mac_address);
      digitalWrite(ledPin2, LOW);
    }
    else
    {
      if (!is_state_given)
      {
        executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &broadcast_state_query);
      }
      else if (is_state_given)
      {
        digitalWrite(ledPin2, HIGH);
        updateMachineState();
        executeAfter(PERIOD_TWO_MS, &elapsed_time_two_ms, &sample_data);
        executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &aggregate_one_second_data);
        executeAfter(PERIOD_TEN_SECONDS, &elapsed_time_ten, &aggregate_ten_second_data);
      }
    }
  }
  else
  {
    digitalWrite(ledPin1, LOW);
  }
}