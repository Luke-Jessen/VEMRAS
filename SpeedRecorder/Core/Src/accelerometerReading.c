/*
 * accelerometerReading.c
 *
 *  Created on: Nov 19, 2025
 *      Author: c0529513
 */


#include "WSEN_ITDS_SELF_TEST_EXAMPLE.h"
#include "WSEN_ITDS_2533020201601.h"
#include "stm32f4xx_hal.h"
#include "accelerometerReading.h"
#include "platform.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


static WE_sensorInterface_t itds;
extern SPI_HandleTypeDef hspi2;



bool ITDS_init(void)
{
    /* Initialize sensor interface (spi with ITDS address, burst mode activated) */
    ITDS_getDefaultInterface(&itds);
    itds.interfaceType = WE_spi;
    itds.options.spi.burstMode = 1;
    itds.handle = &hspi2;

    /* Wait for boot */
    HAL_Delay(50);
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&itds))
    {
    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    HAL_Delay(5);

    /* First communication test */
    uint8_t deviceIdValue = 0;
    if (WE_SUCCESS == ITDS_getDeviceID(&itds, &deviceIdValue))
    {
        if (deviceIdValue == ITDS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-ITDS! */
        {
            debugPrintln("**** ITDS_DEVICE_ID_VALUE: OK ****");
        }
        else
        {
            debugPrintln("**** ITDS_DEVICE_ID_VALUE: NOT OK ****");
            return false;
        }
    }
    else
    {
        debugPrintln("**** ITDS_getDeviceID(): NOT OK ****");
        return false;
    }

    return true;
}


void ITDS_startHighPerformanceMode(void)
{
    debugPrintln("Starting high performance mode...");

    ITDS_state_t dataReady = ITDS_disable;

    /* Enable high performance mode */
    ITDS_setOperatingMode(&itds, ITDS_highPerformance);
    /* Sampling rate of 200 Hz */
    ITDS_setOutputDataRate(&itds, ITDS_odr9);
    /* Enable block data update */
    ITDS_enableBlockDataUpdate(&itds, ITDS_enable);
    /* Enable address auto increment */
    ITDS_enableAutoIncrement(&itds, ITDS_enable);
    /* Filter bandwidth = ODR/2 */
    ITDS_setFilteringCutoff(&itds, ITDS_outputDataRate_2);
    /* Full scale +-16g */
    ITDS_setFullScale(&itds, ITDS_sixteenG);


        /* Wait until the value is ready to read */
        do
        {
            ITDS_isAccelerationDataReady(&itds, &dataReady);
        } while (dataReady == ITDS_disable);

        /* Below, you'll find examples for reading acceleration values of all axes in [mg],
     * as float. Note that as an alternative, there are also
     * functions to get the values for single axes or to get the raw, unconverted values. */



            float xAcc, yAcc, zAcc;
            if (ITDS_getAccelerations_float(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
            {
                debugPrintAcceleration_float("X", xAcc);
                debugPrintAcceleration_float("Y", yAcc);
                debugPrintAcceleration_float("Z", zAcc);

                float accSum = sqrtf((xAcc * xAcc) + (yAcc * yAcc) + (zAcc * zAcc));
                debugPrintAcceleration_float("sum", accSum);
            }
            else
            {
                debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
            }


}

float getAcceleration(float *accelBuff){
/* Wait until the value is ready to read */
	ITDS_state_t dataReady = ITDS_disable;
	do
	{
		ITDS_isAccelerationDataReady(&itds, &dataReady);
	} while (dataReady == ITDS_disable);

/* Below, you'll find examples for reading acceleration values of all axes in [mg],
* as float. Note that as an alternative, there are also
* functions to get the values for single axes or to get the raw, unconverted values. */

	float xAcc, yAcc, zAcc;
	float accSum;


	if (ITDS_getAccelerations_float(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
	{
		debugPrintAcceleration_float("X", xAcc);
		debugPrintAcceleration_float("Y", yAcc);
		debugPrintAcceleration_float("Z", zAcc);

		accelBuff[0] = xAcc;
		accelBuff[1] = yAcc;
		accelBuff[2] = zAcc;

		accSum = sqrtf((xAcc * xAcc) + (yAcc * yAcc) + (zAcc * zAcc));
		debugPrintAcceleration_float("sum", accSum);
	}
	else
	{
		debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
	}

	return accSum;
}


float getAngle(){
 return 0.0;
}




static bool ITDS_discardOldData(void)
{
    ITDS_state_t dataReady;
    ITDS_isAccelerationDataReady(&itds, &dataReady);

    if (dataReady == ITDS_enable)
    {
        int16_t xRawAcc, yRawAcc, zRawAcc;
        if (WE_FAIL == ITDS_getRawAccelerations(&itds, 1, &xRawAcc, &yRawAcc, &zRawAcc))
        {
            return false;
        }
    }
    return true;
}

static void debugPrintAcceleration_float(char axis[], float acc)
{
    acc /= 1000.0f;
    float accAbs = fabs(acc);
    uint16_t full = (uint16_t)accAbs;
    uint16_t decimals = (uint16_t)(((uint32_t)(accAbs * 10000)) % 10000); /* 4 decimal places */

    char bufferFull[6];     /* max 3 pre-decimal point positions */
    char bufferDecimals[5]; /* 4 decimal places */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%04u", decimals);

    debugPrint("ITDS acceleration (float) ");
    debugPrint(axis);
    debugPrint(" = ");
    if (acc < 0)
    {
        debugPrint("-");
    }
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln(" g");
}

