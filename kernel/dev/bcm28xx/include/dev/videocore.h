// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#define VCORE_TAG_REQUEST 0x00000000
#define VCORE_ENDTAG	  0x00000000

#define VCORE_TAG_GET_FIRMWARE_REV			0x00000001
#define VCORE_TAG_GET_FIRMWARE_REV_REQ_LEN	0x00000000
#define VCORE_TAG_GET_FIRMWARE_REV_RSP_LEN	0x00000004

#define VCORE_MAILBOX_FULL	0x80000000
#define VCORE_MAILBOX_EMPTY	0x40000000

#define VC_FB_CHANNEL 0x01
#define ARM_TO_VC_CHANNEL 0x08
#define VC_TO_ARM_CHANNEL 0x09

#define VCORE_SUCCESS			0
#define VCORE_ERR_MBOX_FULL 	1
#define VCORE_ERR_MBOX_TIMEOUT 	2

#define VCORE_READ_ATTEMPTS 0xffffffff


typedef struct {
	uint32_t phys_width;	//request
	uint32_t phys_height;	//request
	uint32_t virt_width;	//request
	uint32_t virt_height;	//request
	uint32_t pitch;			//response
	uint32_t depth;			//request
	uint32_t virt_x_offs;	//request
	uint32_t virt_y_offs;	//request
	uint32_t fb_p;			//response
	uint32_t fb_size;		//response
} fb_mbox_t;

uint32_t get_vcore_framebuffer(fb_mbox_t * fb_mbox);
uint32_t _get_vcore_single(uint32_t tag, uint32_t req_len, uint8_t * rsp, uint32_t rsp_len);

