/*
 * moving_average.c
 *
 *  Created on: May 30, 2024
 *      Author: udaykumar
 */


#include "moving_average.h"

void initMovingAverage(MovingAverage *mAvg) {
    for (int i = 0; i < MOVING_AVERAGE_LENGTH; i++) {
        mAvg->buffer[i] = 0;
    }
    mAvg->counter = 0;
    mAvg->sum = 0;
    mAvg->out = 0;
}

void updateMovingAverage(MovingAverage *mAvg, uint32_t newValue) {
    mAvg->sum -= mAvg->buffer[mAvg->counter];
    mAvg->buffer[mAvg->counter] = newValue;
    mAvg->sum += newValue;

    mAvg->counter++;
    if (mAvg->counter >= MOVING_AVERAGE_LENGTH) {
        mAvg->counter = 0;
    }

    mAvg->out = mAvg->sum / MOVING_AVERAGE_LENGTH;
}
