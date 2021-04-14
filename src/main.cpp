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

char *machineState = "ON";

int currentState = 1; //assume machine machineState is ON initially
int lastState;

String stateMinThreshold;
String stateMaxThreshold;

MPU9250 mpu;
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long current_time = 0.0;
unsigned long elapsed_time_ten;
unsigned long elapsed_time_one;
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
bool is_mac_verified = true;
bool is_state_given = false;
StaticJsonDocument<400> JSONDocument;
float buff_ten_sec[BUFFER_SIZE_TEN_SEC];
float buff_five_min[BUFFER_SIZE_FIVE_MIN];

void led_init();
void setup_wifi();
void setup_mqtt();
void sample_data();
void espclient_reconnect();
void aggregate_one_second_data();
void aggregate_ten_second_data();
void aggregate_five_minute_data();
void send_true_off();
void executeAfter(int, unsigned long, void (*callback)(void));
void handleMqttRequestResponse(char *, byte *, unsigned int);
char *get_data_endpoint();
bool sendMessage(JsonObject, char *);
JsonObject prepareDataPayload(double, double, double, int, double);
JsonObject prepareBroadcastPayload();
JsonObject prepareBroadcastStatePayload();
JsonObject prepareCurrentStatePayload();

void led_init()
{
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
}

void setup_wifi()
{
  WiFiManager wm;
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

void sample_data()
{
  is_mpu_available = mpu.update();
  if (is_mpu_available && buff_counter <= BUFFER_SIZE)
  {
    double total = sqrt(pow(mpu.getAccX(), 2) + pow(mpu.getAccY(), 2));
    buff[buff_counter] = total;
    buff_counter++;
  }
}

void broadcast_mac_address()
{
  sendMessage(prepareBroadcastPayload(), "iot-vib/broadcast");
}

void broadcast_state_query()
{
  sendMessage(prepareBroadcastStatePayload(), "iot-vib/broadcast/state");
}

void aggregate_one_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_counter; i++)
  {
    mean += buff[i] * 100;
  }
  mean = (mean / buff_counter);
  buff_counter = 0;
  if (buff_ten_sec_counter <= BUFFER_SIZE_TEN_SEC)
  {
    buff_ten_sec[buff_ten_sec_counter] = mean;
    buff_ten_sec_counter++;
  }
  Serial.print('One second mean data =');
  Serial.println(mean);

  sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 1, mean), "iot-vib/data");
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
  if (String(topic) == "iot-vib/broadcast/verify")
  {
    Serial.print("message received ");
    Serial.println(messageTemp);
    StaticJsonDocument<200> doc;
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
    else
    {
      is_mac_verified = false;
    }
  }

  if (String(topic) == "iot-vib/broadcast/state/verify" && is_mac_verified)
  {
    Serial.print("Threshold is:");
    Serial.println(messageTemp);
    StaticJsonDocument<200> state;
    DeserializationError err = deserializeJson(state, messageTemp);
    if (err)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(err.f_str());
      return;
    }
    String deviceMacIdState = state["deviceMACId"];
    String minThreshold = state["minThreshold"];
    String maxThreshold = state["maxThreshold"];
    bool stateVerified = state["stateVerified"];
    Serial.println(maxThreshold);
    stateMinThreshold = minThreshold;
    stateMaxThreshold = maxThreshold;
    if (deviceMacIdState == WiFi.macAddress() && stateVerified)
    {
      is_state_given = true;
    }
    else 
    {
      is_state_given = false;
    }
  }
}

void aggregate_ten_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_ten_sec_counter; i++)
  {
    mean += buff_ten_sec[i];
  }
  if (buff_five_min_counter <= BUFFER_SIZE_FIVE_MIN)
  {
    buff_five_min[buff_five_min_counter] = mean;
    buff_five_min_counter++;
  }
  mean = (mean / buff_ten_sec_counter);
  Serial.print("10 second Mean data ");
  Serial.println(mean);
  buff_ten_sec_counter = 0;
  sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 10, mean), "iot-vib/data");
}

void aggregate_five_minute_data()
{
  double mean = 0;
  for (int i = 0; i < buff_five_min_counter; i++)
  {
    mean += buff_five_min[i];
  }
  mean = (mean / buff_five_min_counter);
  Serial.print("5 minute Mean data ");
  Serial.println(mean);
  buff_five_min_counter = 0;
  sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 300, mean), "iot-vib/data");
  lastState = currentState;
  Serial.println("Five minute mean");
  Serial.println(mean);
  if(mean < stateMaxThreshold.toDouble())
  {
    currentState = 0; //washing machine is OFF
  }
  else
  {
    currentState = 1;  //washing machine is ON
  }
}

void send_true_off()
{
  sendMessage(prepareCurrentStatePayload(), "iot-vib/current-state/five-s");
}

void espclient_reconnect()
{

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client"))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      client.subscribe("iot-vib/broadcast/verify");
      client.subscribe("iot-vib/broadcast/state/verify");
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

JsonObject prepareDataPayload(double ax, double ay, double az, int period, double mean)
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["ax"] = ax;
  jsonObject["ay"] = ay;
  jsonObject["az"] = az;
  jsonObject["mean"] = mean;
  jsonObject["period"] = period;
  return jsonObject;
}

JsonObject prepareBroadcastPayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["isVerified"] = is_mac_verified;
  return jsonObject;
}

JsonObject prepareBroadcastStatePayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["isStateGiven"] = is_state_given;
  return jsonObject;
}

JsonObject prepareCurrentStatePayload()
{
  JsonObject jsonObject = JSONDocument.to<JsonObject>();
  jsonObject["deviceMACId"] = WiFi.macAddress();
  jsonObject["state"] = machineState;
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
  setup_mqtt();
  current_time = millis();
  mpu.setup(0x68);
  delay(5000);
  mpu.calibrateAccelGyro();
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
      executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &broadcast_mac_address);
      digitalWrite(ledPin2, LOW);
    }
    else
    {
      if (!is_state_given)
      {
        executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &broadcast_state_query);
      }
      else if(is_state_given)
      {
        digitalWrite(ledPin2, HIGH);
        executeAfter(PERIOD_TWO_MS, &elapsed_time_two_ms, &sample_data);
        executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &aggregate_one_second_data);
        executeAfter(PERIOD_TEN_SECONDS, &elapsed_time_ten, &aggregate_ten_second_data);
        executeAfter(PERIOD_FIVE_MINUTES, &elapsed_time_fivemins, &aggregate_five_minute_data);
        if (lastState == 0 && currentState == 0)
        {
          machineState = "TRUEOFF";
          executeAfter(PERIOD_FIVE_SECOND, &elapsed_time_five, &send_true_off);
        }
        else
        {
          machineState = "ON";
          executeAfter(PERIOD_FIVE_SECOND, &elapsed_time_five, &send_true_off);
        }
      }
    }
  }
  else
  {
    digitalWrite(ledPin1, LOW);
  }
}