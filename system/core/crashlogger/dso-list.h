// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <magenta/types.h>

#include "utils.h"

typedef struct dsoinfo {
    struct dsoinfo* next;
    mx_vaddr_t base;
    char buildid[MAX_BUILDID_SIZE * 2 + 1];
    bool debug_file_tried;
    mx_status_t debug_file_status;
    char* debug_file;
    char name[];
} dsoinfo_t;

extern dsoinfo_t* dso_fetch_list(mx_handle_t h, const char* name);

extern void dso_free_list(dsoinfo_t*);

extern dsoinfo_t* dso_lookup (dsoinfo_t* dso_list, mx_vaddr_t pc);

extern void dso_print_list(dsoinfo_t* dso_list);

extern mx_status_t dso_find_debug_file(dsoinfo_t* dso, const char** out_debug_file);
