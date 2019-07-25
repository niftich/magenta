// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "osboot.h"

#include <stdio.h>
#include <string.h>
#include <xefi.h>

#include <magenta/pixelformat.h>

static efi_guid AcpiTableGUID = ACPI_TABLE_GUID;
static efi_guid Acpi2TableGUID = ACPI_20_TABLE_GUID;
static uint8_t ACPI_RSD_PTR[8] = "RSD PTR ";

uint64_t find_acpi_root(efi_handle img, efi_system_table* sys) {
    efi_configuration_table* cfgtab = sys->ConfigurationTable;
    int i;

    for (i = 0; i < sys->NumberOfTableEntries; i++) {
        if (!xefi_cmp_guid(&cfgtab[i].VendorGuid, &AcpiTableGUID) &&
            !xefi_cmp_guid(&cfgtab[i].VendorGuid, &Acpi2TableGUID)) {
            // not an ACPI table
            continue;
        }
        if (memcmp(cfgtab[i].VendorTable, ACPI_RSD_PTR, 8)) {
            // not the Root Description Pointer
            continue;
        }
        return (uint64_t)cfgtab[i].VendorTable;
    }
    return 0;
}

static void get_bit_range(uint32_t mask, int* high, int* low) {
    *high = -1;
    *low = -1;
    int idx = 0;
    while (mask) {
        if (*low < 0 && (mask & 1)) *low = idx;
        idx++;
        mask >>= 1;
    }
    *high = idx - 1;
}

static int get_mx_pixel_format_from_bitmask(efi_pixel_bitmask bitmask) {
    int r_hi = -1, r_lo = -1, g_hi = -1, g_lo = -1, b_hi = -1, b_lo = -1;

    get_bit_range(bitmask.RedMask, &r_hi, &r_lo);
    get_bit_range(bitmask.GreenMask, &g_hi, &g_lo);
    get_bit_range(bitmask.BlueMask, &b_hi, &b_lo);

    if (r_lo < 0 || g_lo < 0 || b_lo < 0) {
        goto unsupported;
    }

    if ((r_hi == 23 && r_lo == 16) &&
        (g_hi == 15 && g_lo == 8) &&
        (b_hi == 7 && b_lo == 0)) {
        return MX_PIXEL_FORMAT_RGB_x888;
    }

    if ((r_hi == 7 && r_lo == 5) &&
        (g_hi == 4 && g_lo == 2) &&
        (b_hi == 1 && b_lo == 0)) {
        return MX_PIXEL_FORMAT_RGB_332;
    }

    if ((r_hi == 15 && r_lo == 11) &&
        (g_hi == 10 && g_lo == 5) &&
        (b_hi == 4 && b_lo == 0)) {
        return MX_PIXEL_FORMAT_RGB_565;
    }

    if ((r_hi == 7 && r_lo == 6) &&
        (g_hi == 5 && g_lo == 4) &&
        (b_hi == 3 && b_lo == 2)) {
        return MX_PIXEL_FORMAT_RGB_2220;
    }

unsupported:
    printf("unsupported pixel format bitmask: r %08x / g %08x / b %08x\n",
            bitmask.RedMask, bitmask.GreenMask, bitmask.BlueMask);
    return MX_PIXEL_FORMAT_NONE;
}

uint32_t get_mx_pixel_format(efi_graphics_output_protocol* gop) {
    efi_graphics_pixel_format efi_fmt = gop->Mode->Info->PixelFormat;
    switch (efi_fmt) {
    case PixelBlueGreenRedReserved8BitPerColor:
        return MX_PIXEL_FORMAT_RGB_x888;
    case PixelBitMask:
        return get_mx_pixel_format_from_bitmask(gop->Mode->Info->PixelInformation);
    default:
        printf("unsupported pixel format %d!\n", efi_fmt);
        return MX_PIXEL_FORMAT_NONE;
    }
}
