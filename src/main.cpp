#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "MPU9250.h"
#include <WiFi.h>

#define SDA 21
#define SCL 22
const int PERIOD_TEN_SECONDS = 10000;
const int PERIOD_ONE_SECOND = 1000;
const int PERIOD_TWO_MS = 2;
const int BUFFER_SIZE = 200;
const int BUFFER_SIZE_TEN_SEC = 20;
const int BAUD_RATE = 115200;
// Replace the next variables with your SSID/Password combination
const char *ssid = "****";
const char *password = "****";
const char *mqtt_server = "10.42.0.1";
const int mqtt_port = 1883;

char *data_endpoint = "iot-vib/data";
char *broadcast_endpoint = "iot-vib/broadcast";

String device_mac_address;

MPU9250 mpu;
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long current_time = 0.0;
unsigned long elapsed_time_ten;
unsigned long elapsed_time_one;
unsigned long elapsed_time_two_ms;

int buff_counter = 0;
float buff[BUFFER_SIZE];
bool wire_status = false;
int buff_ten_sec_counter = 0;
bool is_mpu_available = false;
bool is_client_connected = false;
bool is_mac_verified = true;
StaticJsonDocument<400> JSONDocument;
float buff_ten_sec[BUFFER_SIZE_TEN_SEC];

void setup_wifi();
void setup_mqtt();
void sample_data();
void espclient_reconnect();
void aggregate_one_second_data();
void aggregate_ten_second_data();
void executeAfter(int, unsigned long, void (*callback)(void));
void handleMqttRequestResponse(char *, byte *, unsigned int);
char *get_data_endpoint();
bool sendMessage(JsonObject, char *);
JsonObject prepareDataPayload(double, double, double, int, double);
JsonObject prepareBroadcastPayload();

void setup_wifi()
{
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  device_mac_address = WiFi.macAddress();
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

int get_buff_ten_sec_len()
{
  return sizeof buff_ten_sec / sizeof *buff_ten_sec;
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

void aggregate_one_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_counter; i++)
  {
    mean += buff[i];
  }
  mean = (mean / buff_counter) * 100;
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

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
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
}

void aggregate_ten_second_data()
{
  double mean = 0;
  for (int i = 0; i < buff_ten_sec_counter; i++)
  {
    mean += buff_ten_sec[i];
  }
  mean = (mean / buff_ten_sec_counter) * 100;
  Serial.print("10 second Mean data ");
  Serial.println(mean);
  buff_ten_sec_counter = 0;
  sendMessage(prepareDataPayload(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), 10, mean), "iot-vib/data");
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

bool sendMessage(JsonObject jsonObject, char *endpoint)
{
  char JSONmessageBuffer[200];
  serializeJson(jsonObject, JSONmessageBuffer);
  if (client.publish(endpoint, JSONmessageBuffer) == true)
  {
    // Serial.println("Success sending message");
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
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  wire_status = Wire.begin();
  while (!Serial || !wire_status)
  {
  }
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
    current_time = millis();
    if (!is_mac_verified)
    {
      executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &broadcast_mac_address);
    }
    else
    {
      executeAfter(PERIOD_TWO_MS, &elapsed_time_two_ms, &sample_data);
      executeAfter(PERIOD_ONE_SECOND, &elapsed_time_one, &aggregate_one_second_data);
      executeAfter(PERIOD_TEN_SECONDS, &elapsed_time_ten, &aggregate_ten_second_data);
    }
  }
}