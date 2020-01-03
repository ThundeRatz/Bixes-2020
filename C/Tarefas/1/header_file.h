#ifndef __HEADER_FILE_H__
#define __HEADER_FILE_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define LS_AMOUNT 4
#define FIRST_SENSOR 0
#define LAST_SENSOR 4
#define FILTER_VALUE 0.1

#define LONG_DISTANCE 120
#define SHORT_DISTANCE 2500

#define ARC_FIRST_TURN_TIME_MS 280
#define ARC_SECOND_TURN_TIME_MS 95
#define STRAIGHT_MS 140
#define START_SPEED 13

#define ADC_BUFFER_SIZE 128
#define ADC1_CHANNELS 4

#define SENSOR_THRESHOLD 1500

#define LINE_VALUE 2000
#define DISTANCE_VALUE 700

#define REVERSION_TURN_SPEED 50
#define REVERSION_REVERT_TIME 100
#define REVERSION_TURN_TIME 100

#define SWING_PERIOD 2000
#define SWING_SPEED 15
#define PULSE_SPEED 15

#define DS_AMOUNT 5
#define DS_MAX_RANGE 500

#define MIN_SPEED -100
#define MAX_SPEED 100

#define abs(x) ((x) > 0 ? (x) : -(x))
#define max(x, y) ((x) > (y) ? (x) : (y))
#define min(x, y) ((x) < (y) ? (x) : (y))
#define constrain(v, x, y) ((v) < (x) ? (x) : ((v) > (y) ? (y) : (v)))

volatile uint32_t sensors[LS_AMOUNT];

typedef enum _sensor_position {
    LS_RIGHT,
    DS_LEFT,
    DS_RIGHT,
    LS_LEFT,
} sensor_position_t;

typedef enum _line_sensor_position {
    LS_FRONT_LEFT,
    LS_BACK_LEFT,
    LS_FRONT_RIGHT,
    LS_BACK_RIGHT,
} line_sensor_position_t

typedef enum _distance_sensor_position {
    DS_SIDE_LEFT,
    DS_FRONT_LEFT,
    DS_FRONT_CENTER,
    DS_FRONT_RIGHT,
    DS_SIDE_RIGHT
} distance_sensor_position_t;

motors_init(void);
void motors(int8_t speed_left, int8_t speed_right);
void distance_sensors_init(void);
void distance_sensors_update(void);
uint16_t distance_sensor_get(distance_sensor_position_t sensor);
void rozeta_strategy_set(uint8_t strategy);
void onix_strategy_set(uint8_t strategy);
void rozeta_strategy_run(void);
void onix_strategy_run(void);
void line_sensors_init(void);
int map(int former_value, int former_min, int former_max, int new_min, int new_max);
uint16_t distance_sensor_get(distance_sensor_position_t sensor);
uint8_t last_bt_recv(void);
void motors_straight_line(int8_t speed_left, int8_t speed_right);
uint16_t get_timer(uint16_t timer);
void reset_timer(uint16_t* timer);
bool is_line_sensor_high(line_sensor_position_t sensor);
bool line_see();
void test_preparation(void);


 // __HEADER_FILE_H__
