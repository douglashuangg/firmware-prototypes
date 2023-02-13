/*
 * optical_flow.c
 *
 *  Created on: Jan 30, 2023
 *      Author: hdoug
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Inc/optical_flow.h"

opflow_t opflow;

static bool opflowIsCalibrating = false;
static timeMs_t opflowCalibrationStartedAt;
static float opflowCalibrationBodyAcc;
static float opflowCalibrationFlowAcc;

// opFlow detect that detects uses MSP

// sets rate of change of body
static void opflowZeroBodyGyroAcc(void)
{
    opflow.gyroBodyRateTimeUs = 0;
    opflow.gyroBodyRateAcc[X] = 0;
    opflow.gyroBodyRateAcc[Y] = 0;
}

// INITIALIZATION
bool opflowInit(void)
{
	// information is passed to opflow.dev
    if (!opflowDetect(&opflow.dev, opticalFlowConfig()->opflow_hardware)) {
        return false;
    }

    if (!opflow.dev.initFn(&opflow.dev)) {
        sensorsClear(SENSOR_OPFLOW);
        return false;
    }

    opflowZeroBodyGyroAcc();

    return true;
}



// NOT COPY PASTED
int read_flow_data(){
	// we don't know how much data is available ...
	// does msp just send a whole package of data? ...
	uint8_t buffer[4];
	if (HAL_UART_Receive(&huart2, buffer, 4, 100) != HAL_OK) {
		return -1;
	}

}


static bool opflowDetect(opflowDev_t * dev, uint8_t opflowHardwareToUse)
{
    // opticalFlowSensor_e opflowHardware = OPFLOW_NONE;
    // requestedSensors[SENSOR_INDEX_OPFLOW] = opflowHardwareToUse;

#if defined(USE_OPFLOW_MSP)
	// this actually sets the values
	if (virtualOpflowDetect(dev, &opflowMSPVtable)) {
		opflowHardware = OPFLOW_MSP;
	}
#endif

    // detectedSensors[SENSOR_INDEX_OPFLOW] = opflowHardware;
    // sensorsSet(SENSOR_OPFLOW);
    return true;
}

// CALIBRATION
void opflowStartCalibration(void)
{
    opflowCalibrationStartedAt = millis();
    opflowIsCalibrating = true;
    opflowCalibrationBodyAcc = 0;
    opflowCalibrationFlowAcc = 0;
}

// UPDATE FUNCTION IMPORTANT!!!!
void opflowUpdate(timeUs_t currentTimeUs)
{
    if (!opflow.dev.updateFn)
        return;

    if (opflow.dev.updateFn(&opflow.dev)) {
        // Indicate valid update
        opflow.isHwHealty = true;
        opflow.lastValidUpdate = currentTimeUs;
        opflow.rawQuality = opflow.dev.rawData.quality;

        // Handle state switching
        switch (opflow.flowQuality) {
            case OPFLOW_QUALITY_INVALID:
                if (opflow.dev.rawData.quality >= OPFLOW_SQUAL_THRESHOLD_HIGH) {
                    opflow.flowQuality = OPFLOW_QUALITY_VALID;
                }
                break;

            case OPFLOW_QUALITY_VALID:
                if (opflow.dev.rawData.quality <= OPFLOW_SQUAL_THRESHOLD_LOW) {
                    opflow.flowQuality = OPFLOW_QUALITY_INVALID;
                }
                break;
        }

        // Opflow updated. Assume zero values unless further processing sets otherwise
        opflow.flowRate[X] = 0;
        opflow.flowRate[Y] = 0;
        opflow.bodyRate[X] = 0;
        opflow.bodyRate[Y] = 0;

        // In the following code we operate deg/s and do conversion to rad/s in the last step
        // Calculate body rates
        if (opflow.gyroBodyRateTimeUs > 0) {
            opflow.bodyRate[X] = opflow.gyroBodyRateAcc[X] / opflow.gyroBodyRateTimeUs;
            opflow.bodyRate[Y] = opflow.gyroBodyRateAcc[Y] / opflow.gyroBodyRateTimeUs;
        }

        // If quality of the flow from the sensor is good - process further
        if (opflow.flowQuality == OPFLOW_QUALITY_VALID) {
            const float integralToRateScaler = (opticalFlowConfig()->opflow_scale > 0.01f) ? (1.0e6 / opflow.dev.rawData.deltaTime) / (float)opticalFlowConfig()->opflow_scale : 0.0f;

            // Apply sensor alignment
            applySensorAlignment(opflow.dev.rawData.flowRateRaw, opflow.dev.rawData.flowRateRaw, opticalFlowConfig()->opflow_align);

            // Calculate flow rate and accumulated body rate
            opflow.flowRate[X] = opflow.dev.rawData.flowRateRaw[X] * integralToRateScaler;
            opflow.flowRate[Y] = opflow.dev.rawData.flowRateRaw[Y] * integralToRateScaler;

            // Only update DEBUG_FLOW_RAW if flow is good
            DEBUG_SET(DEBUG_FLOW_RAW, 0, (opflow.flowRate[X]));
            DEBUG_SET(DEBUG_FLOW_RAW, 1, (opflow.flowRate[Y]));
            DEBUG_SET(DEBUG_FLOW_RAW, 2, (opflow.bodyRate[X]));
            DEBUG_SET(DEBUG_FLOW_RAW, 3, (opflow.bodyRate[Y]));
        }

        // Process calibration
        if (opflowIsCalibrating) {
            // Blink LED
            LED0_TOGGLE;

            if ((millis() - opflowCalibrationStartedAt) > OPFLOW_CALIBRATE_TIME_MS) {
                // Finish calibration if we accumulated more than 3600 deg of rotation over 30 seconds
                if (opflowCalibrationBodyAcc > 3600.0f) {
                    opticalFlowConfigMutable()->opflow_scale = opflowCalibrationFlowAcc / opflowCalibrationBodyAcc;
                    saveConfigAndNotify();
                }

                opflowIsCalibrating = 0;
            }
            else if (opflow.flowQuality == OPFLOW_QUALITY_VALID) {
                // Ongoing calibration - accumulate body and flow rotation magniture if opflow quality is good enough
                const float invDt = 1.0e6 / opflow.dev.rawData.deltaTime;
                opflowCalibrationBodyAcc += calc_length_pythagorean_2D(opflow.bodyRate[X], opflow.bodyRate[Y]);
                opflowCalibrationFlowAcc += calc_length_pythagorean_2D(opflow.dev.rawData.flowRateRaw[X], opflow.dev.rawData.flowRateRaw[Y]) * invDt;
            }
        }

        // Convert to radians so NAV doesn't have to do the conversion
        opflow.bodyRate[X] = DEGREES_TO_RADIANS(opflow.bodyRate[X]);
        opflow.bodyRate[Y] = DEGREES_TO_RADIANS(opflow.bodyRate[Y]);
        opflow.flowRate[X] = DEGREES_TO_RADIANS(opflow.flowRate[X]);
        opflow.flowRate[Y] = DEGREES_TO_RADIANS(opflow.flowRate[Y]);

        // Zero out gyro accumulators to calculate rotation per flow update
        opflowZeroBodyGyroAcc();
    }
    else {
        // No new data available
        if (opflow.isHwHealty && ((currentTimeUs - opflow.lastValidUpdate) > OPFLOW_UPDATE_TIMEOUT_US)) {
            opflow.isHwHealty = false;

            opflow.flowQuality = OPFLOW_QUALITY_INVALID;
            opflow.rawQuality = 0;

            opflow.flowRate[X] = 0;
            opflow.flowRate[Y] = 0;
            opflow.bodyRate[X] = 0;
            opflow.bodyRate[Y] = 0;

            opflowZeroBodyGyroAcc();
        }
    }
}
