#include <stdint.h>
#include "MSP_format.h"

uint8_t msp_serial_checksum(uint8_t checksum, const uint8_t *data, uint32_t len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}


uint32_t msp_serial_send_frame(msp_port_t *msp, const uint8_t * hdr, uint32_t hdr_len, const uint8_t * data, uint32_t data_len, const uint8_t * crc, uint32_t crc_len)
{
    if (msp->uart == nullptr) {
        return 0;
    }

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const uint32_t total_frame_length = hdr_len + data_len + crc_len;

    if (!msp->uart->tx_pending() && (msp->uart->txspace() < total_frame_length)) {
        return 0;
    }

    // Transmit frame
    msp->uart->write(hdr, hdr_len);
    msp->uart->write(data, data_len);
    msp->uart->write(crc, crc_len);

    return total_frame_length;
}

uint32_t msp_serial_encode(msp_port_t *msp, msp_packet_t *packet, msp_version_e msp_version, bool is_request)
{
    static const uint8_t msp_magic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER;
    /*
        Note: after calling sbuf_switch_to_reader() sbuf_bytes_remaining() returns the size of the packet
    */
    const uint16_t data_len = sbuf_bytes_remaining(&packet->buf);

    // This code is needed
    uint8_t code;
    if (is_request) {
        code = '<';
    } else if (packet->result == MSP_RESULT_ERROR) {
        code = '!';
    } else {
        code = '>';
    }
    // header values $X<
    const uint8_t hdr_buf[16] = { '$', msp_magic[msp_version], code };
    // checksum values
    uint8_t crc_buf[2];
    uint32_t hdr_len = 3;
    uint32_t crc_len = 0;

#define V1_CHECKSUM_STARTPOS 3
    if (msp_version == MSP_V1) {
        msp_header_v1_t * hdr_v1 = (msp_header_v1_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v1_t);
        hdr_v1->cmd = packet->cmd;

        // Add JUMBO-frame header if necessary
        if (data_len >= JUMBO_FRAME_SIZE_LIMIT) {
            msp_header_jumbo_t * hdr_jumbo = (msp_header_jumbo_t *)&hdr_buf[hdr_len];
            hdr_len += sizeof(msp_header_jumbo_t);

            hdr_v1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdr_jumbo->size = data_len;
        } else {
            hdr_v1->size = data_len;
        }

        // Pre-calculate CRC
        crc_buf[crc_len] = msp_serial_checksum_buf(0, hdr_buf + V1_CHECKSUM_STARTPOS, hdr_len - V1_CHECKSUM_STARTPOS);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;
    } else if (msp_version == MSP_V2_OVER_V1) {
        msp_header_v1_t * hdr_v1 = (msp_header_v1_t *)&hdr_buf[hdr_len];

        hdr_len += sizeof(msp_header_v1_t);

        msp_header_v2_t * hdr_v2 = (msp_header_v2_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v2_t);

        const uint32_t v1_payload_size = sizeof(msp_header_v2_t) + data_len + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdr_v1->cmd = MSP_V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1_payload_size >= JUMBO_FRAME_SIZE_LIMIT) {
            msp_header_jumbo_t * hdr_jumbo = (msp_header_jumbo_t *)&hdr_buf[hdr_len];
            hdr_len += sizeof(msp_header_jumbo_t);

            hdr_v1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdr_jumbo->size = v1_payload_size;
        } else {
            hdr_v1->size = v1_payload_size;
        }

        // Fill V2 header
        hdr_v2->flags = packet->flags;
        hdr_v2->cmd = packet->cmd;
        hdr_v2->size = data_len;

        // V2 CRC: only V2 header + data payload
        crc_buf[crc_len] = crc8_dvb_s2_update(0, (uint8_t *)hdr_v2, sizeof(msp_header_v2_t));
        crc_buf[crc_len] = crc8_dvb_s2_update(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;

        // V1 CRC: All headers + data payload + V2 CRC byte
        crc_buf[crc_len] = msp_serial_checksum_buf(0, hdr_buf + V1_CHECKSUM_STARTPOS, hdr_len - V1_CHECKSUM_STARTPOS);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], crc_buf, crc_len);
        crc_len++;
    } else if (msp_version == MSP_V2_NATIVE) {
    	// this is the only code that matters
        msp_header_v2_t * hdr_v2 = (msp_header_v2_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v2_t);

        hdr_v2->flags = packet->flags;
        hdr_v2->cmd = packet->cmd;
        hdr_v2->size = data_len;

        crc_buf[crc_len] = crc8_dvb_s2_update(0, (uint8_t *)hdr_v2, sizeof(msp_header_v2_t));
        crc_buf[crc_len] = crc8_dvb_s2_update(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;
    } else {
        // Shouldn't get here
        return 0;
    }

    // Send the frame
    return msp_serial_send_frame(msp, hdr_buf, hdr_len, sbuf_ptr(&packet->buf), data_len, crc_buf, crc_len);
}
