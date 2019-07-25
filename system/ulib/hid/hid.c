// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <hid/hid.h>
#include <hid/usages.h>

#include <string.h>
#include <strings.h>

#define KEYSET(bitmap,n) (bitmap[(n) >> 5] |= (1 << ((n) & 31)))
#define KEYCLR(bitmap,n) (bitmap[(n) >> 5] &= ~(1 << ((n) & 31)))

void hid_kbd_parse_report(uint8_t buf[8], hid_keys_t* keys) {
    memset(keys, 0, sizeof(hid_keys_t));
    // modifiers start at bit 224
    keys->keymask[7] = buf[0];
    for (int i = 2; i < 8; i++) {
        if (buf[i] == 0) break;
        KEYSET(keys->keymask, buf[i]);
    }
}

void hid_kbd_pressed_keys(const hid_keys_t* prev, const hid_keys_t* cur, hid_keys_t* pressed) {
    memset(pressed, 0, sizeof(hid_keys_t));
    for (int i = 0; i < 8; i++) {
        pressed->keymask[i] = (prev->keymask[i] ^ cur->keymask[i]) & cur->keymask[i];
    }
}

void hid_kbd_released_keys(const hid_keys_t* prev, const hid_keys_t* cur, hid_keys_t* released) {
    memset(released, 0, sizeof(hid_keys_t));
    for (int i = 0; i < 8; i++) {
        released->keymask[i] = (prev->keymask[i] ^ cur->keymask[i]) & prev->keymask[i];
    }
}

uint8_t hid_kbd_next_key(hid_keys_t* keys) {
    for (int i = 0; i < 8; i++) {
        int key = ffs(keys->keymask[i]);
        if (key) {
            key += i*32 - 1;
            KEYCLR(keys->keymask, key);
            return key;
        }
    }
    return 0;
}

uint8_t hid_map_key(uint32_t usage, bool shift, keychar_t* keymap) {
    if (usage > HID_USAGE_KEY_KP_DOT) return 0;
    if (shift) {
        return keymap[usage].shift_c;
    } else {
        return keymap[usage].c;
    }
}
