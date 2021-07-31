#include <Arduino.h>
#include "./ota/OTA.h"
#include "./config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "CREDENTIALS"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <uDebugLib.h>

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

bool pump_current_state = 0;
bool heater_current_state = 0;



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
  pinMode(HEAT_PUMP_RELAY, OUTPUT);     digitalWrite(HEAT_PUMP_RELAY, LOW);
  pinMode(MAIN_PUMP_SSR, OUTPUT);     digitalWrite(MAIN_PUMP_SSR, LOW);


  #ifdef DEBUG
    Serial.begin(115200, SERIAL_8N1);
  #endif
  
  // Start the DS18B20 sensor
  sensors.begin();
  
  if(USE_OTA_UPDATE) {
    setupOTA("POOL_CTRL", SSID, PASSWORD);
  }

  DEBUG_PRINTLN("Connecting to WiFi");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin (ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINT(".");
      delay(500);
  }

  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINT("IP address:\t");
  DEBUG_PRINTLN(WiFi.localIP());      

  previousMillis = millis();
  DEBUG_PRINTLN("Setup done");
}

void loop() {
    unsigned long currentMillis = millis();

      // Used to slowdown the process to the number of milliseconds defined in variable interval (see config.h file)
    if (currentMillis - previousMillis >= SCAN_TIME_INTERVAL) {
        previousMillis = currentMillis; 

        // READ AND TRANSMIT SENSORS
        readSensors();
        transmitSensors();
        
        // SET PUMP AND HEATER STATES
        setPumpState(pump_current_state);
        setHeaterState(heater_current_state);

        // TRANSMIT STATES
        transmitStates();

    }

    // Process MQTT Subscriptions
    client.loop();
}

/*****************************************************************************************
 * Read the sensor values (WATER_TEMP_SENSOR, AMBIANT_AIR_TEMP_SENSOR, CASE_TEMP_SENSOR)
 *****************************************************************************************/
int8_t readSensors() {

  sensors.requestTemperatures(); 
  temperatureC = sensors.getTempCByIndex(0);
  temperatureF = sensors.getTempFByIndex(0);
  DEBUG_PRINT(temperatureC);
  DEBUG_PRINTLN("ºC");
  DEBUG_PRINT(temperatureF);
  DEBUG_PRINTLN("ºF");

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
    client.publish(STAT_TEMP_WATER_TOPIC, message);
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
  digitalWrite(MAIN_PUMP_SSR, pump_state);
  return 1;
}


/*****************************************************************************************
 * Set the heater running state based on parameter value (from server via MQTT)
 *****************************************************************************************/
int8_t setHeaterState(bool heater_state) {
  
  digitalWrite(HEAT_PUMP_RELAY, heater_state);
  return 1;
}


/*****************************************************************************************
 * Set the heater running state based on parameter value (from server via MQTT)
 *****************************************************************************************/
int8_t transmitStates() {
  char message[3];
  sprintf(message, "%d", pump_current_state);

  if (!client.connected()) {
    reconnect();
  }

  client.publish(STAT_PUMP_TOPIC, message);
  sprintf(message, "%d", heater_current_state);
  client.publish(STAT_HEATER_TOPIC, message);
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

    payload[length] = '\0';
    char *message = (char *) payload;

    DEBUG_PRINTLN(topic);
    DEBUG_PRINTLN(message);

    if (strcmp(topic, CMD_PUMP_TOPIC ) ==0 ) {
      pump_current_state = atoi(message);
    }
    if (strcmp(topic, CMD_HEATER_TOPIC ) ==0 ) {
      heater_current_state = atoi(message);
    }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    if (client.connect("POOL_CTRL")) {
      DEBUG_PRINTLN("connected");
      // Once connected, publish an announcement...
      client.publish(STAT_SERVICE_TOPIC, "I am live again");
      // ... and resubscribe
      client.subscribe(CMD_PUMP_TOPIC);
      client.subscribe(CMD_HEATER_TOPIC);
    } else {
      DEBUG_PRINT("failed, rc = ");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}