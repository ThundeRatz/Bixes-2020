/**
 * @file utils.c
 *
 * @brief Utility functions
 */

#include "utils.h"

/*****************************************
 * Public Function Body Definitions
 *****************************************/

int map(int former_value, int former_min, int former_max, int new_min, int new_max) {
    long int new_value;

    new_value = (long int) (former_value - former_min) * (new_max - new_min);
    new_value /= (former_max - former_min);
    new_value += new_min;

    return (int) new_value;
}
