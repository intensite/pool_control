#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H


#define DEBUG 1
#define SCAN_TIME_INTERVAL 5000


// PIN ASSIGNMENT 
#define HEAT_PUMP_RELAY 19
#define MAIN_PUMP_SSR 17
#define STATUS_LED 25

#define WATER_TEMP_SENSOR 32
#define AIR_TEMP_SENSOR 33
// #define CASE_TEMP_SENSOR 21

// #define SPARE_SENSOR_1 21
// #define SPARE_SENSOR_2 21
// #define SPARE_SENSOR_3 21

// #define SDA 21 
// #define SCL 22 

// MQTT STUFF
#define STAT_TEMP_WATER_TOPIC "pool/stat/temp/water"
#define STAT_TEMP_AIR_TOPIC "pool/stat/temp/air"
#define STAT_TEMP_SYSTEM_TOPIC "pool/stat/temp/system"
#define STAT_PUMP_TOPIC "pool/stat/pump"
#define STAT_HEATER_TOPIC "pool/stat/heater"
#define STAT_SERVICE_TOPIC "pool/stat/service"
#define CMD_PUMP_TOPIC "pool/cmd/pump"
#define CMD_HEATER_TOPIC "pool/cmd/heater"





#endif // CONFIG_FILE_H1