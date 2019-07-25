// Copyright 2017 The Fuchsia Authors
// Copyright (c) 2016, Google, Inc. All rights reserved
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <arch.h>
#include <arch/arm64/mp.h>

#define PSCI64_PSCI_VERSION                 (0x84000000)
#define PSCI64_CPU_SUSPEND                  (0xC4000001)
#define PSCI64_CPU_OFF                      (0x84000002)
#define PSCI64_CPU_ON                       (0xC4000003)
#define PSCI64_AFFINITY_INFO                (0xC4000004)
#define PSCI64_MIGRATE                      (0xC4000005)
#define PSCI64_MIGRATE_INFO_TYPE            (0x84000006)
#define PSCI64_MIGRATE_INFO_UP_CPU          (0xC4000007)
#define PSCI64_SYSTEM_OFF                   (0x84000008)
#define PSCI64_SYSTEM_RESET                 (0x84000009)
#define PSCI64_PSCI_FEATURES                (0x8400000A)
#define PSCI64_CPU_FREEZE                   (0x8400000B)
#define PSCI64_CPU_DEFAULT_SUSPEND          (0xC400000C)
#define PSCI64_NODE_HW_STATE                (0xC400000D)
#define PSCI64_SYSTEM_SUSPEND               (0xC400000E)
#define PSCI64_PSCI_SET_SUSPEND_MODE        (0x8400000F)
#define PSCI64_PSCI_STAT_RESIDENCY          (0xC4000010)
#define PSCI64_PSCI_STAT_COUNT              (0xC4000011)

/* TODO NOTE: - currently these routines assume cpu topologies that are described only in AFF0 and AFF1.
            If a system is architected such that AFF2 or AFF3 are non-zero then this code will need
            to be revisited
*/

typedef uint64_t (*psci_call_proc)(ulong arg0, ulong arg1, ulong arg2, ulong arg3);

extern psci_call_proc do_psci_call;

static inline uint32_t psci_get_version(void) {

    return (uint32_t)do_psci_call(PSCI64_PSCI_VERSION,0,0,0);
}

/* powers down the calling cpu - only returns if call fails */
static inline uint32_t psci_cpu_off(void) {

    return (uint32_t)do_psci_call(PSCI64_CPU_OFF,0,0,0);
}

static inline uint32_t psci_cpu_on(uint64_t cluster, uint64_t cpuid, paddr_t entry) {

    return (uint32_t)do_psci_call(PSCI64_CPU_ON, ARM64_MPID(cluster, cpuid), entry, 0);
}

static inline uint32_t psci_get_affinity_info(uint64_t cluster, uint64_t cpuid) {

    return (uint32_t)do_psci_call(PSCI64_AFFINITY_INFO, ARM64_MPID(cluster, cpuid), 0, 0);
}

static inline void psci_system_off(void) {

    do_psci_call(PSCI64_SYSTEM_OFF, 0, 0, 0);
}

static inline void psci_system_reset(void) {

    do_psci_call(PSCI64_SYSTEM_RESET, 0, 0, 0);
}
