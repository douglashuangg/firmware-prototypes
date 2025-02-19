/*
 * optical_flow.h
 *
 *  Created on: Jan 30, 2023
 *      Author: hdoug
 */

#ifndef INC_OPTICAL_FLOW_H_
#define INC_OPTICAL_FLOW_H_

#include <stdint.h>
#include "MSP_format.h"


// USEFUL
typedef struct raw_opflow_data {
	uint8_t quality;
	uint32_t motion_x;
	uint32_t motion_y;
} raw_opflow_data_t;


// define registers

// INITIALIZATION

// DATA ACQUISITION

// LOW-LEVEL FUNCTIONS

// bool opflowInit(void);
// void opflowUpdate(timeUs_t currentTimeUs);
void opFlowUpdate(msp_format_t *raw_data);

void read_flow_data();

void process_opflow(uint8_t * rx_data);



#endif /* INC_OPTICAL_FLOW_H_ */
