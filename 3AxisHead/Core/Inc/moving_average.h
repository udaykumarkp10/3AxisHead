/*
 * moving_average.h
 *
 *  Created on: May 30, 2024
 *      Author: udaykumar
 */


#ifndef INC_MOVING_AVERAGE_H_
#define INC_MOVING_AVERAGE_H_

#include <project.h>

#define MOVING_AVERAGE_LENGTH 15

typedef struct {
    uint32_t buffer[MOVING_AVERAGE_LENGTH];
    uint16_t counter;
    uint32_t out;
    uint64_t sum;
} MovingAverage;

void initMovingAverage(MovingAverage *mAvg);
void updateMovingAverage(MovingAverage *mAvg, uint32_t newValue);

#endif /* INC_MOVING_AVERAGE_H_ */
