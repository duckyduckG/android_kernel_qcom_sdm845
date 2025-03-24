/*
 * Copyright (c) 2018 Light Labs, Inc.
 * All Rights Reserved
 * Proprietary and Confidential - Light Labs, Inc.
 */

#ifndef __MSM_LIGHT_MESSAGING_H__
#define __MSM_LIGHT_MESSAGING_H__

typedef enum
{
	LCC_CMD_REQ_MSG,
	LCC_CMD_RESP_MSG,
	LCC_CMD_DUMMY_MSG,
	INTER_ASIC_EVENT_MSG,
	PREVIEW_AE_SETTINGS_UPDATE_MSG = 7,
	TRANSFER_DESCRIPTOR_MSG = 8,
	AF_UPDATE_MSG = 9,
	NUM_LIGHT_MSG_TYPES
} light_msg_type_e;

#define LIGHT_MAX_MESSAGE_SIZE 512

typedef struct  __attribute__ ((packed))
{
    uint16_t msgLength;
    uint16_t address;
    uint8_t  data[0];
} light_i2c_lcc_t;

typedef struct  __attribute__ ((packed))
{
    uint16_t msgLength;
    uint8_t  data[0];
} light_i2c_lcc_response_t;

typedef uint16_t light_msg_type_t;

typedef struct  __attribute__ ((packed))
{
	light_msg_type_t msg_type;
	uint16_t length;
} light_msg_header_t;

typedef struct __attribute__ ((packed))
{
	uint16_t tid;
	uint8_t status;
	uint8_t source;
} lcc_cmd_resp_t;

typedef struct  __attribute__ ((packed))
{
	light_msg_header_t msg_header;
	lcc_cmd_resp_t slave_resp;
} lcc_resp_msg_t;

typedef enum
{
	START_READ_REQUEST,
	CONTINUE_READ_REQUEST = 0x0100 /* Byte order is swapped by msm_camera_qup_i2c utilities */
} light_i2c_read_request_e;

typedef struct __attribute__ ((packed))
{
	uint16_t req;
} light_i2c_read_request_t;

#endif /* __MSM_LIGHT_MESSAGING_H__ */
