/*
 * acceleromterReading.h
 *
 *  Created on: Nov 19, 2025
 *      Author: c0529513
 */

#ifndef INC_ACCELEROMETERREADING_H_
#define INC_ACCELEROMETERREADING_H_

#include <stdbool.h>

bool ITDS_init(void);

void ITDS_startHighPerformanceMode(void);

float getAcceleration(float *accelBuff);


float getAngle();

static bool ITDS_discardOldData(void);

static void debugPrintAcceleration_float(char axis[], float acc);
#endif /* INC_ACCELEROMETERREADING_H_ */
