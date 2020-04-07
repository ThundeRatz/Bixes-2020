#include stdint.h
#include time.h

#include "header_file.h"

#if defined (_WIN32)
#include <windows.h>
#define delay(ms) Sleep((ms))
#else
#include <unistd.h>
#define delay(ms) usleep((ms) * 1000.0)
#endif

#define TIM_PERIOD 500

static uint8_t bt_recv;
static uint16_t distance_sensors[DS_AMOUNT] = {0};
static uint32_t line_sensors[LS_AMOUNT];
static const distance_sensor_position_t __sensors[DS_AMOUNT] = {DS_SIDE_RIGHT, DS_FRONT_RIGHT, DS_FRONT_CENTER,
                                                                DS_FRONT_LEFT, DS_SIDE_LEFT};

void motors(int8_t speed_left, int8_t speed_right) {
    speed_left = constrain(speed_left, MIN_SPEED, MAX_SPEED);
    speed_right = constrain(speed_right, MIN_SPEED, MAX_SPEED);

    speed_left = abs(speed_left);
    speed_right = abs(speed_right);

    speed_left = map(speed_left, 0, MAX_SPEED, 0, TIM_PERIOD);
    speed_right = map(speed_right, 0, MAX_SPEED, 0, TIM_PERIOD);
}

int map(int former_value, int former_min, int former_max, int new_min, int new_max) {
    long int new_value;

    new_value = (long int) (former_value - former_min) * (new_max - new_min);
    new_value /= (former_max - former_min);
    new_value += new_min;

    return (int) new_value;
}

uint8_t last_bt_recv(void) {
    return bt_recv;
}

void motors_straight_line(int8_t speed_left, int8_t speed_right) {
    motors(speed_left,0);
    motors(0,speed_right);
}

volatile bool adc1_reading_done;

uint32_t get_sensor_value(sensor_position_t sensor) {
    return sensors[sensor];
}

bool line_see(sensor_position_t sensor){
    if sensors[sensor] < LINE_VALUE {
        return true;
    }
    return false;
}

bool distance_see(sensor_position_t sensor){
    if(sensors[sensor] > DISTANCE_VALUE){
        return true;
    }
    return false;
}

uint16_t get_timer(uint16_t timer) {
    if(timer == 0){
        timer = 1000;
    }
    else{
        timer = 0;
    }
    return timer;
}

void reset_timer(uint16_t* timer){
    *timer = 0;
}

uint16_t distance_sensor_get(distance_sensor_position_t sensor) {
    return distance_sensors[(int) sensor];
}

bool is_line_sensor_high(line_sensor_position_t sensor) {
    return (line_sensors[(int) sensor] < 1000);
}

void distance_sensors_update(void) {
    static uint16_t ranges[LAST_SENSOR] = {0};

    for (int i = FIRST_SENSOR; i < LAST_SENSOR; i++) {

            distance_sensors[__sensors[i]] =
                FILTER_VALUE * ranges[i] + (1 - FILTER_VALUE) * distance_sensors[__sensors[i]];
    }
}

void test_preparation(void){
    printf("\nObrigado pela confiança! \nEm alguns momentos estaremos de volta com os resultados do teste.\n");
    for(int i, i < 5, i++){
        printf(".\n");
        delay(1000);
    }
}
