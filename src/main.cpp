#include <Arduino.h>
#include "./ota/OTA.h"
#include "./config.h"


unsigned long previousMillis = 0;

int8_t readSensors();
int8_t transmitSensors();
int8_t getServerInstructions();
int8_t setPumpState(bool pump_state);
int8_t setHeaterState(bool heater_state);
int8_t transmitStates();


void setup() {
  // put your setup code here, to run once:
  pinMode(HEAT_PUMP_RELAY, OUTPUT);     digitalWrite(HEAT_PUMP_RELAY, LOW);
  pinMode(MAIN_PUMP_SSR, OUTPUT);     digitalWrite(MAIN_PUMP_SSR, LOW);
}

void loop() {
    unsigned long currentMillis = millis();

      // Used to slowdown the process to the number of milliseconds defined in variable interval (see config.h file)
    if (currentMillis - previousMillis >= SCAN_TIME_INTERVAL) {
        previousMillis = currentMillis; 

        // READ SENSORS
        // TRANSMIT SENSORS
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
  return 1;
}


/*****************************************************************************************
 * Send the send the sensor values to the server via MQTT
 *****************************************************************************************/
int8_t transmitSensors() {
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