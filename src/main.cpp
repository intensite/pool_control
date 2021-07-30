#include <Arduino.h>
#include "./ota/OTA.h"
#include "./config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "CREDENTIALS"
#include <OneWire.h>
#include <DallasTemperature.h>


unsigned long previousMillis = 0;

int8_t readSensors();
int8_t transmitSensors();
int8_t getServerInstructions();
int8_t setPumpState(bool pump_state);
int8_t setHeaterState(bool heater_state);
int8_t transmitStates();
int8_t setMQTTConnection();
void reconnect();
void mqtt_callback(char* topic, byte* payload, unsigned int length);

OneWire oneWire(WATER_TEMP_SENSOR);
DallasTemperature sensors(&oneWire);
float temperatureC = 0.0;
float temperatureF = 0.0;


// WiFi
WiFiClient wifiClient;
#ifdef __credentials__
    char ssid[]      = SSID;               // Set you WiFi SSID
    char password[]  = PASSWORD;           // Set you WiFi password
#else
    char *ssid      = "";               // Set you WiFi SSID
    char *password  = "";               // Set you WiFi password
#endif

//MQTT
IPAddress server(192, 168, 70, 233);
PubSubClient client(server, 1883, mqtt_callback, wifiClient);


void setup() {
  // put your setup code here, to run once:
  pinMode(HEAT_PUMP_RELAY, OUTPUT);     digitalWrite(HEAT_PUMP_RELAY, LOW);
  pinMode(MAIN_PUMP_SSR, OUTPUT);     digitalWrite(MAIN_PUMP_SSR, LOW);

  Serial.begin(115200, SERIAL_8N1);
  
  // Start the DS18B20 sensor
  sensors.begin();
  
  if(USE_OTA_UPDATE)
    setupOTA("POOL_CTRL", SSID, PASSWORD);

    Serial.println("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin (ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());      


  // SETUP MQTT
  Serial.println("Connecting to MQTT");
  if (client.connect("POOL_CTRL")) {
    Serial.println("Connected...");
    char str[40];
    strcpy(str,"POOL_CTRL live at: ");
    strcat(str,  WiFi.localIP().toString().c_str());
    Serial.println("Sending ststus on SERVICETOPIC...");
      client.publish(SERVICETOPIC, str);
      // client.subscribe(COMMANDTOPIC);
  }
      previousMillis = millis();
    Serial.println("Setup done");
}

void loop() {
    unsigned long currentMillis = millis();

      // Used to slowdown the process to the number of milliseconds defined in variable interval (see config.h file)
    if (currentMillis - previousMillis >= SCAN_TIME_INTERVAL) {
        previousMillis = currentMillis; 

        // READ SENSORS
        readSensors();
        // TRANSMIT 
        transmitSensors();
        // GET INSTRUCTION MESSAGES

        // SET PUMP STATE (MESSAGE.PUMP.STATE)
        // SET HEATER STATE (MESSAGE.HEATER.STATE)

        // TRANSMIT STATES

    }
}

/*****************************************************************************************
 * Read the sensor values (WATER_TEMP_SENSOR, AMBIANT_AIR_TEMP_SENSOR, CASE_TEMP_SENSOR)
 *****************************************************************************************/
int8_t readSensors() {

  sensors.requestTemperatures(); 
  temperatureC = sensors.getTempCByIndex(0);
  temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");

  return 1;
}


/*****************************************************************************************
 * Send the send the sensor values to the server via MQTT
 *****************************************************************************************/
int8_t transmitSensors() {

  char message[10];
  sprintf(message, "%f", temperatureC);

    if (!client.connected()) {
      reconnect();
    }
    client.publish(WATER_TEMP_TOPIC, message);
  return 1;
}


/*****************************************************************************************
 * Query the server via MQTT  (probably in a callback)
 *****************************************************************************************/
int8_t getServerInstructions() {
  return 1;
}


/*****************************************************************************************
 * Set the pump running state based on parameter value (from server via MQTT)
 *****************************************************************************************/
int8_t setPumpState(bool pump_state) {
  return 1;
}


/*****************************************************************************************
 * Set the heater running state based on parameter value (from server via MQTT)
 *****************************************************************************************/
int8_t setHeaterState(bool heater_state) {
  return 1;
}


/*****************************************************************************************
 * Set the heater running state based on parameter value (from server via MQTT)
 *****************************************************************************************/
int8_t transmitStates() {
  return 1;
}


/*****************************************************************************************
 * Set the MQTT connection
 *****************************************************************************************/
int8_t setMQTTConnection() {
  return 1;
}

// Process the received MQTT Messages
void mqtt_callback(char* topic, byte* payload, unsigned int length) {








  // digitalWrite(STATUS_LED, HIGH);
  
  //   String IRcommand = "";
  //   int i = 0;
  //   while (payload[i] > 0) {
  //       IRcommand = IRcommand + (char)payload[i];
  //       i++;
  //   }
  //   DynamicJsonDocument json(300);
  //   auto error = deserializeJson(json, payload);

  //   // Test if parsing succeeds.
  //   if (error) {
  //       Serial.print(F("deserializeJson() failed with code "));
  //       Serial.println(error.c_str());
  //       return;
  //   }

  //   // Check if json contains an array
  //   JsonArray command_array = json.as<JsonArray>();

  //   if(command_array) {

  //     Serial.println("Seems to be an array of commands ");

  //     // using C++11 syntax (preferred):
  //     for (JsonVariant value : command_array) {
  //         JsonObject command = value.as<JsonObject>();
  //         unsigned long type = command["type"];
  //         unsigned long valu =  strtoul(command["value"], (char**)0, 0);  // Conver from string to ulong
          
  //         int repeat = command["repeat"];

  //         // Serial.print("type ");
  //         // Serial.println(type);
  //         // Serial.print("value ");
  //         // Serial.println(valu);
  //         // Serial.print("repeat ");
  //         // Serial.println(repeat);

  //       //  if(type == 99) {

  //       //  }

  //         sendCode(type, valu, repeat);
  //         delay(MULTICODE_DELAY);

  //     }
  //   } else {

  //     int type = json["type"];
  //     // unsigned long valu = json["value"];
  //     unsigned long valu =  strtoul(json["value"], (char**)0, 0);  // Conver from string to ulong
  //     int repeat = json["repeat"];
  //     // Serial.print("Payload ");
  //     // Serial.println(IRcommand);
  //     // Serial.print("type ");
  //     // Serial.println(type);
  //     // Serial.print("value ");
  //     // Serial.println(valu);
  //     // Serial.print("repeat ");
  //     // Serial.println(repeat);
  //     sendCode(type, valu, repeat);
  //   }
  //   digitalWrite(STATUS_LED, LOW);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("POOL_CTRL")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(SERVICETOPIC, "I am live again");
      // ... and resubscribe
      //  client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}