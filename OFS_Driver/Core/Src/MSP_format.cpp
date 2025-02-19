#include <stdint.h>
#include "MSP_format.h"


uint8_t MSP_HEADER_START = 0x24;



uint8_t msp_serial_checksum(uint8_t checksum, const uint8_t *data, uint32_t len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}


uint8_t crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed)
{
    crc ^= a;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ seed;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

bool msp_parse_received_data(msp_format_t *msp, uint8_t * raw_data){
	if(raw_data[0] != MSP_HEADER_START){
		return false;
	}
	// Processing is MSP V2
	uint8_t offset = 8;

	msp->checksum1 = 0;
	msp->cmd_flags = raw_data[3];
	msp->id = (raw_data[4] << 8 | raw_data[5]);
	msp->data_size = (raw_data[6] << 8 | raw_data[7]);
	for(int i = 3; i < 8; i++){
		msp->checksum1 = crc8_dvb(msp->checksum1, raw_data[i], 0xD5);
	}

	for(int i = 0; i < (msp->data_size/0x100) ; i++){ // 0x100 is 256 in hex
		msp->payload[i] = raw_data[offset++];
		msp->checksum1 = crc8_dvb(msp->checksum1, raw_data[offset++], 0xD5);
	}
	if(msp->checksum1 == raw_data[offset]){
		msp->c_state = MSP_COMMAND_RECEIVED;
	} else {
		msp->c_state = MSP_IDLE;
	}
	return true;
}




