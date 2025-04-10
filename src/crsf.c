#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/prctl.h>

#include "crsf.h"
#include "osd.h"

// Global variables for CRSF data
float crsf_alt = 0.0;
float crsf_battery_voltage = 0.0;
float crsf_battery_current = 0.0;
float crsf_battery_mah = 0.0;
float crsf_remaining = 0.0;
float crsf_pitch = 0.0;
float crsf_roll = 0.0;
float crsf_yaw = 0.0;
char crsf_flight_mode[256] = "";

// CRSF port configuration
int crsf_port = 2001; // Default port, can be changed via command line
int crsf_thread_signal = 0;

// CRC8 calculation functions
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc <<= 1;
        }
    }
    return crc & 0xFF;
}

uint8_t crc8_data(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

int crsf_validate_frame(const uint8_t* frame, size_t length) {
    if (length < 4) {
        return 0; // Not enough data
    }
    uint8_t calculated_crc = crc8_data(frame + 2, length - 3);
    uint8_t received_crc = frame[length - 1];
    return calculated_crc == received_crc;
}

void handleCrsfPacket(uint8_t ptype, const uint8_t* data, size_t length) {
    if (ptype == CRSF_BATTERY_SENSOR) {
        crsf_battery_voltage = (data[3] << 8 | data[4]) / 10.0;
        crsf_battery_current = (data[5] << 8 | data[6]) / 10.0;
        crsf_battery_mah = (float)((data[7] << 16) | (data[8] << 8) | data[9]);
        crsf_remaining = (float)(data[10]);
        
        // Update OSD with battery information
        osd_tag tags[2];
        strcpy(tags[0].key, "sysid");
        snprintf(tags[0].val, sizeof(tags[0].val), "%d", 1);
        strcpy(tags[1].key, "compid");
        snprintf(tags[1].val, sizeof(tags[1].val), "%d", 1);
        
        void *batch = osd_batch_init(4);
        osd_add_double_fact(batch, "battery.voltage", tags, 2, (double)crsf_battery_voltage);
        osd_add_double_fact(batch, "battery.current", tags, 2, (double)crsf_battery_current);
        osd_add_double_fact(batch, "battery.consumed_mah", tags, 2, (double)crsf_battery_mah);
        osd_add_double_fact(batch, "battery.remaining_percent", tags, 2, (double)crsf_remaining);
        osd_publish_batch(batch);
    } 
    else if (ptype == CRSF_ATTITUDE_TYPE) {
        crsf_pitch = (int16_t)(data[3] << 8 | data[4]) * CONVERSION_FACTOR;
        crsf_roll = (int16_t)(data[5] << 8 | data[6]) * CONVERSION_FACTOR;
        crsf_yaw = (int16_t)(data[7] << 8 | data[8]) * CONVERSION_FACTOR;
        
        // Update OSD with attitude information
        osd_tag tags[2];
        strcpy(tags[0].key, "sysid");
        snprintf(tags[0].val, sizeof(tags[0].val), "%d", 1);
        strcpy(tags[1].key, "compid");
        snprintf(tags[1].val, sizeof(tags[1].val), "%d", 1);
        
        void *batch = osd_batch_init(3);
        osd_add_double_fact(batch, "attitude.pitch", tags, 2, (double)crsf_pitch);
        osd_add_double_fact(batch, "attitude.roll", tags, 2, (double)crsf_roll);
        osd_add_double_fact(batch, "attitude.yaw", tags, 2, (double)crsf_yaw);
        osd_publish_batch(batch);
    } 
    else if (ptype == CRSF_FRAMETYPE_FLIGHT_MODE) {
        int mode_length = data[1] - 3;
        if (mode_length > 0 && mode_length < 256) {
            memcpy(crsf_flight_mode, &data[3], mode_length);
            crsf_flight_mode[mode_length] = '\0'; // Null terminate the string
            
            // Simplify flight mode names
            if (strstr(crsf_flight_mode, "ERR") != NULL) {
                memcpy(crsf_flight_mode, "no info", 8);
            } else if (strstr(crsf_flight_mode, "ACRO") != NULL) {
                memcpy(crsf_flight_mode, "acro", 5);
            } else if (strstr(crsf_flight_mode, "STAB") != NULL) {
                memcpy(crsf_flight_mode, "stab", 5);
            }
            
            // Update OSD with flight mode information
            osd_tag tags[2];
            strcpy(tags[0].key, "sysid");
            snprintf(tags[0].val, sizeof(tags[0].val), "%d", 1);
            strcpy(tags[1].key, "compid");
            snprintf(tags[1].val, sizeof(tags[1].val), "%d", 1);
            
            void *batch = osd_batch_init(1);
            osd_add_str_fact(batch, "flight_mode", tags, 2, crsf_flight_mode);
            osd_publish_batch(batch);
        } else {
            strcpy(crsf_flight_mode, "unknown");
        }
    } 
    else if (ptype == CRSF_FRAMETYPE_BARO_ALTITUDE) {
        int16_t raw_altitude = (int16_t)(data[3] << 8 | data[4]); // Big Endian value

        if (raw_altitude & 0x8000) { // Check sign bit
            // Altitude is already in meters
            crsf_alt = (int16_t)(raw_altitude & 0x7FFF);
        } else {
            // Altitude is in decimeters
            crsf_alt = (int16_t)((raw_altitude - 10000) / 10);
        }
        
        // Update OSD with altitude information
        osd_tag tags[2];
        strcpy(tags[0].key, "sysid");
        snprintf(tags[0].val, sizeof(tags[0].val), "%d", 1);
        strcpy(tags[1].key, "compid");
        snprintf(tags[1].val, sizeof(tags[1].val), "%d", 1);
        
        void *batch = osd_batch_init(1);
        osd_add_double_fact(batch, "altitude", tags, 2, (double)crsf_alt);
        osd_publish_batch(batch);
    }
}

void* __CRSF_THREAD__(void* arg) {
    pthread_setname_np(pthread_self(), "__CRSF");
    printf("Starting CRSF thread...\n");
    
    // Create socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("ERROR: Unable to create CRSF socket: %s\n", strerror(errno));
        return 0;
    }

    // Bind port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(crsf_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("ERROR: Unable to bind CRSF port: %s\n", strerror(errno));
        close(sock);
        return 0;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("ERROR: Unable to set CRSF rx timeout: %s\n", strerror(errno));
    }

    uint8_t input[4096];
    size_t input_len = 0;

    while (!crsf_thread_signal) {
        ssize_t len_received = recv(sock, input + input_len, sizeof(input) - input_len, 0);
        if (len_received > 0) {
            input_len += len_received;

            while (input_len > 2) {
                size_t expected_len = input[1] + 2;
                if (expected_len > 64 || expected_len < 4) {
                    printf("Invalid packet length: %zu. Resetting buffer.\n", expected_len);
                    input_len = 0;
                } else if (input_len >= expected_len) {
                    uint8_t single[64];
                    memcpy(single, input, expected_len);
                    memmove(input, input + expected_len, input_len - expected_len);
                    input_len -= expected_len;

                    if (!crsf_validate_frame(single, expected_len)) {
                        printf("CRC error\n");
                    } else {
                        handleCrsfPacket(single[2], single, expected_len);
                    }
                } else {
                    break; // Not enough data for a complete packet, wait for more
                }
            }
        } else if (len_received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("Error receiving data");
            }
        }
        usleep(1); // Small delay to prevent CPU hogging
    }
    
    printf("CRSF thread done.\n");
    close(sock);
    return 0;
} 