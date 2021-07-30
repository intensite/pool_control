#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H


#define DEBUG 1
#define SCAN_TIME_INTERVAL 5000


// PIN ASSIGNMENT 
#define STATUS_LED 16
#define PIEZO_BUZZER 27
#define HEAT_PUMP_RELAY 19
#define MAIN_PUMP_SSR 17
#define STATUS_LED 16

#define WATER_TEMP_SENSOR 32
// #define AMBIANT_AIR_TEMP_SENSOR 21
// #define CASE_TEMP_SENSOR 21

// #define SPARE_SENSOR_1 21
// #define SPARE_SENSOR_2 21
// #define SPARE_SENSOR_3 21

// #define SDA 21 
// #define SCL 22 

// MQTT STUFF
#define WATER_TEMP_TOPIC "POOL/water"
#define COMMANDTOPIC "POOL/command"
#define SERVICETOPIC "POOL/service"


#endif // CONFIG_FILE_H1