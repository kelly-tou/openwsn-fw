#ifndef __USENSOR_NETWORK_H
#define __USENSOR_NETWORK_H

#include <stdint.h>
#include "config.h"
#include "opentimers.h"
#include "udp.h"

#define USENSOR_NETWORK_PERIOD_MS 3000

typedef enum {
    SENSOR_INVALID = -1,
    SENSOR_NONE = 0,
    SENSOR_POTENTIOMETRIC
} sensor_type_e;

typedef struct __attribute__((packed)) {
    uint16_t data;
} potentiometric_sensor_data_t;

typedef struct __attribute__((packed)) {
    sensor_type_e sensor_type;
    union {
        potentiometric_sensor_data_t potentiometric;
    } data;
} sensor_payload_t;

typedef struct __attribute__((packed)) {
    char preamble;
    uint16_t addr_16b;
    sensor_payload_t payload;
} data_packet_t;

void usensor_network_init(void);

#endif
