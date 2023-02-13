/*
 * MSP_format.h
 *
 *  Created on: Feb. 5, 2023
 *      Author: hdoug
 */

#ifndef INC_MSP_FORMAT_H_
#define INC_MSP_FORMAT_H_

#define MSP_VERSION_MAGIC_INITIALIZER { 'M', 'M', 'X' }

typedef struct msp_packet_s {
    sbuf_t buf;
    int16_t cmd;
    uint8_t flags;
    int16_t result;
} msp_packet_t;

typedef struct PACKED {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} msp_header_v2_t;


// checksum before transmit and after receive
uint8_t msp_serial_checksum(uint8_t checksum, const uint8_t *data, uint32_t len);

// encode data to send
uint32_t msp_serial_encode();

// receive response
bool msp_parse_received_data();

// send request
uint8_t msp_serial_send_frame();

// error handling



#endif /* INC_MSP_FORMAT_H_ */
