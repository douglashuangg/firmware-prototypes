/*
 * MSP_format.h
 *
 *  Created on: Feb. 5, 2023
 *      Author: hdoug
 */

#ifndef INC_MSP_FORMAT_H_
#define INC_MSP_FORMAT_H_

#include <stdbool.h>

typedef enum {
    MSP_IDLE,
    MSP_COMMAND_RECEIVED
} msp_state_e;

typedef struct msp_format_s {
    uint8_t payload[192]; // 192 is defined in ardupilot
    uint16_t data_size;
    uint16_t id;
    uint8_t cmd_flags;
    uint16_t cmd_msp;
    uint8_t checksum1;
    msp_state_e c_state;
} msp_format_t;



//typedef struct msp_packet_s {
  //  sbuf_t buf;
  //  int16_t cmd;
 //   uint8_t flags;
  //  int16_t result;
//} msp_packet_t;

typedef struct PACKED {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} msp_header_v2_t;


// checksum before transmit and after receive
uint8_t msp_serial_checksum(uint8_t checksum, const uint8_t *data, uint32_t len);

// encode data to send
// uint32_t msp_serial_encode();

// receive response
bool msp_parse_received_data(msp_format_t *msp, uint8_t * raw_data);


// send request
// uint8_t msp_serial_send_frame();

// error handling



#endif /* INC_MSP_FORMAT_H_ */
