#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

// CRSF frame types
#define CRSF_BATTERY_SENSOR 0x08
#define CRSF_ATTITUDE_TYPE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21
#define CRSF_FRAMETYPE_BARO_ALTITUDE 0x09

// Conversion constants
#define RAD_TO_DEG 57.2957795131
#define CONVERSION_FACTOR (RAD_TO_DEG / 10000.0)

// Global variables for CRSF data
extern float crsf_alt;
extern float crsf_battery_voltage;
extern float crsf_battery_current;
extern float crsf_battery_mah;
extern float crsf_remaining;
extern float crsf_pitch;
extern float crsf_roll;
extern float crsf_yaw;
extern char crsf_flight_mode[256];

// CRSF port configuration
extern int crsf_port;
extern int crsf_thread_signal;

// Function declarations
void* __CRSF_THREAD__(void* arg);
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_data(const uint8_t* data, size_t length);
int crsf_validate_frame(const uint8_t* frame, size_t length);
void handleCrsfPacket(uint8_t ptype, const uint8_t* data, size_t length);

#endif // CRSF_H 