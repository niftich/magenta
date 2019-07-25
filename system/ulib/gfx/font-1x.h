// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FONT1X_WIDTH 9
#define FONT1X_HEIGHT 16

static const uint16_t FONT1X[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7e, 0x81, 0xa5, 0x81, 0x81, 0xbd,
    0x99, 0x81, 0x81, 0x7e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7e, 0xff, 0xdb, 0xff, 0xff, 0xc3,
    0xe7, 0xff, 0xff, 0x7e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x36, 0x7f, 0x7f, 0x7f,
    0x7f, 0x3e, 0x1c, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x1c, 0x3e, 0x7f,
    0x3e, 0x1c, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0xe7, 0xe7,
    0xe7, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0xff, 0xff,
    0x7e, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3c,
    0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xc3,
    0xc3, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x66, 0x42,
    0x42, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0x99, 0xbd,
    0xbd, 0x99, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x78, 0x70, 0x58, 0x4c, 0x1e, 0x33,
    0x33, 0x33, 0x33, 0x1e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x66, 0x66, 0x66, 0x66, 0x3c,
    0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfc, 0xcc, 0xfc, 0x0c, 0x0c, 0x0c,
    0x0c, 0x0e, 0x0f, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfe, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6,
    0xc6, 0xe6, 0xe7, 0x67, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x18, 0xdb, 0x3c, 0xe7,
    0x3c, 0xdb, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x7f, 0x1f,
    0x0f, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x60, 0x70, 0x78, 0x7c, 0x7f, 0x7c,
    0x78, 0x70, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18,
    0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
    0x66, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xfe, 0xdb, 0xdb, 0xdb, 0xde, 0xd8,
    0xd8, 0xd8, 0xd8, 0xd8, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3e, 0x63, 0x06, 0x1c, 0x36, 0x63, 0x63,
    0x36, 0x1c, 0x30, 0x63, 0x3e, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18,
    0x7e, 0x3c, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
    0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x30, 0x7f,
    0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x06, 0x7f,
    0x06, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
    0x03, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x66, 0xff,
    0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x1c, 0x1c, 0x3e,
    0x3e, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x7f, 0x7f, 0x3e, 0x3e,
    0x1c, 0x1c, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x18,
    0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x66, 0x66, 0x66, 0x24, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x36, 0x36, 0x7f, 0x36, 0x36,
    0x36, 0x7f, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00,
    0x18, 0x18, 0x3e, 0x63, 0x43, 0x03, 0x3e, 0x60,
    0x60, 0x61, 0x63, 0x3e, 0x18, 0x18, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x43, 0x63, 0x30, 0x18,
    0x0c, 0x06, 0x63, 0x61, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x36, 0x36, 0x1c, 0x6e, 0x3b,
    0x33, 0x33, 0x33, 0x6e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x0c, 0x0c, 0x06, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x0c,
    0x0c, 0x0c, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x18, 0x0c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3c, 0xff,
    0x3c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e,
    0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x18, 0x18, 0x0c, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x30, 0x18,
    0x0c, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x66, 0xc3, 0xc3, 0xdb, 0xdb,
    0xc3, 0xc3, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x1c, 0x1e, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x60, 0x30, 0x18, 0x0c,
    0x06, 0x03, 0x63, 0x7f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x60, 0x60, 0x3c, 0x60,
    0x60, 0x60, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x30, 0x38, 0x3c, 0x36, 0x33, 0x7f,
    0x30, 0x30, 0x30, 0x78, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x03, 0x03, 0x03, 0x3f, 0x60,
    0x60, 0x60, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x06, 0x03, 0x03, 0x3f, 0x63,
    0x63, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x63, 0x60, 0x60, 0x30, 0x18,
    0x0c, 0x0c, 0x0c, 0x0c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x63, 0x3e, 0x63,
    0x63, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x63, 0x7e, 0x60,
    0x60, 0x60, 0x30, 0x1e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
    0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
    0x00, 0x18, 0x18, 0x0c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0c, 0x06,
    0x0c, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00,
    0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60,
    0x30, 0x18, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x30, 0x18, 0x18,
    0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3e, 0x63, 0x63, 0x7b, 0x7b,
    0x7b, 0x3b, 0x03, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x1c, 0x36, 0x63, 0x63, 0x7f,
    0x63, 0x63, 0x63, 0x63, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3f, 0x66, 0x66, 0x66, 0x3e, 0x66,
    0x66, 0x66, 0x66, 0x3f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x66, 0x43, 0x03, 0x03, 0x03,
    0x03, 0x43, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1f, 0x36, 0x66, 0x66, 0x66, 0x66,
    0x66, 0x66, 0x36, 0x1f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x66, 0x46, 0x16, 0x1e, 0x16,
    0x06, 0x46, 0x66, 0x7f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x66, 0x46, 0x16, 0x1e, 0x16,
    0x06, 0x06, 0x06, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x66, 0x43, 0x03, 0x03, 0x7b,
    0x63, 0x63, 0x66, 0x5c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x63, 0x63, 0x63, 0x63, 0x7f, 0x63,
    0x63, 0x63, 0x63, 0x63, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x78, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x33, 0x33, 0x33, 0x1e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x67, 0x66, 0x66, 0x36, 0x1e, 0x1e,
    0x36, 0x66, 0x66, 0x67, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0f, 0x06, 0x06, 0x06, 0x06, 0x06,
    0x06, 0x46, 0x66, 0x7f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc3, 0xe7, 0xff, 0xff, 0xdb, 0xc3,
    0xc3, 0xc3, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x63, 0x67, 0x6f, 0x7f, 0x7b, 0x73,
    0x63, 0x63, 0x63, 0x63, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x63, 0x63, 0x63,
    0x63, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3f, 0x66, 0x66, 0x66, 0x3e, 0x06,
    0x06, 0x06, 0x06, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x63, 0x63, 0x63,
    0x63, 0x6b, 0x7b, 0x3e, 0x30, 0x70, 0x00, 0x00,
    0x00, 0x00, 0x3f, 0x66, 0x66, 0x66, 0x3e, 0x36,
    0x66, 0x66, 0x66, 0x67, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x63, 0x63, 0x06, 0x1c, 0x30,
    0x60, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xdb, 0x99, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
    0x63, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
    0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xdb,
    0xdb, 0xff, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x18,
    0x3c, 0x66, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18,
    0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xc3, 0x61, 0x30, 0x18, 0x0c,
    0x06, 0x83, 0xc3, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
    0x0c, 0x0c, 0x0c, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0e, 0x1c,
    0x38, 0x70, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3c, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x1c, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00,
    0x0c, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x30, 0x3e,
    0x33, 0x33, 0x33, 0x6e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x06, 0x06, 0x1e, 0x36, 0x66,
    0x66, 0x66, 0x66, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x63, 0x03,
    0x03, 0x03, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x38, 0x30, 0x30, 0x3c, 0x36, 0x33,
    0x33, 0x33, 0x33, 0x6e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x63, 0x7f,
    0x03, 0x03, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x36, 0x26, 0x06, 0x0f, 0x06,
    0x06, 0x06, 0x06, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x3e, 0x30, 0x33, 0x1e, 0x00,
    0x00, 0x00, 0x07, 0x06, 0x06, 0x36, 0x6e, 0x66,
    0x66, 0x66, 0x66, 0x67, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x18, 0x00, 0x1c, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x60, 0x60, 0x00, 0x70, 0x60, 0x60,
    0x60, 0x60, 0x60, 0x60, 0x66, 0x66, 0x3c, 0x00,
    0x00, 0x00, 0x07, 0x06, 0x06, 0x66, 0x36, 0x1e,
    0x1e, 0x36, 0x66, 0x67, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x18, 0x18, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0xff, 0xdb,
    0xdb, 0xdb, 0xdb, 0xdb, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x66, 0x66,
    0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x63, 0x63,
    0x63, 0x63, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x66, 0x66,
    0x66, 0x66, 0x66, 0x3e, 0x06, 0x06, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x3e, 0x30, 0x30, 0x78, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x6e, 0x66,
    0x06, 0x06, 0x06, 0x0f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x63, 0x06,
    0x1c, 0x30, 0x63, 0x3e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x0c, 0x0c, 0x3f, 0x0c, 0x0c,
    0x0c, 0x0c, 0x6c, 0x38, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x33, 0x33,
    0x33, 0x33, 0x33, 0x6e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3,
    0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3,
    0xdb, 0xdb, 0xff, 0x66, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x66, 0x3c,
    0x18, 0x3c, 0x66, 0xc3, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x63, 0x63,
    0x63, 0x63, 0x63, 0x7e, 0x60, 0x30, 0x1f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x33, 0x18,
    0x0c, 0x06, 0x63, 0x7f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0e, 0x18,
    0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18,
    0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0e, 0x18, 0x18, 0x18, 0x70, 0x18,
    0x18, 0x18, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x6e, 0x3b, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x1c, 0x36, 0x63,
    0x63, 0x63, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
};
