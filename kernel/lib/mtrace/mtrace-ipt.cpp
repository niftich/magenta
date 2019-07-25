// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

// N.B. This is ideally temporary. It is used by Intel PT support, and is a
// stopgap until "resources" can be used to read/write x86 MSRs.
// "mtrace" == "magenta trace": the idea being to be a generalization of
// ktrace. It's all temporary, but there may be other uses before the stopgap
// is no longer necessary.

#ifdef __x86_64__ // entire file

#include <inttypes.h>

#include <arch/user_copy.h>
#include "lib/mtrace.h"
#include "trace.h"

#include <magenta/mtrace.h>

#include "arch/x86/proc_trace.h"

#define LOCAL_TRACE 0

status_t mtrace_ipt_control(uint32_t action, uint32_t options,
                            void* arg, uint32_t size) {
    TRACEF("action %u, options 0x%x, arg %p, size 0x%x\n",
           action, options, arg, size);

    switch (action) {
    case MTRACE_IPT_SET_MODE: {
        if (options != 0)
            return ERR_INVALID_ARGS;
        uint32_t mode;
        if (size != sizeof(mode))
            return ERR_INVALID_ARGS;
        if (arch_copy_from_user(&mode, arg, size) != NO_ERROR)
            return ERR_INVALID_ARGS;
        TRACEF("action %u, mode 0x%x\n", action, mode);
        switch (mode) {
        case IPT_MODE_CPUS:
            return x86_ipt_set_mode(IPT_TRACE_CPUS);
        case IPT_MODE_THREADS:
            return x86_ipt_set_mode(IPT_TRACE_THREADS);
        default:
            return ERR_INVALID_ARGS;
        }
    }

    case MTRACE_IPT_STAGE_CPU_DATA: {
        mx_x86_pt_regs_t regs;
        if (size != sizeof(regs))
            return ERR_INVALID_ARGS;
        if (arch_copy_from_user(&regs, arg, size) != NO_ERROR)
            return ERR_INVALID_ARGS;
        uint32_t cpu = MTRACE_IPT_OPTIONS_CPU(options);
        if ((options & ~MTRACE_IPT_OPTIONS_CPU_MASK) != 0)
            return ERR_INVALID_ARGS;
        TRACEF("action %u, cpu %u, ctl 0x%" PRIx64 ", output_base 0x%" PRIx64 "\n",
               action, cpu, regs.ctl, regs.output_base);
        return x86_ipt_stage_cpu_data(cpu, &regs);
    }

    case MTRACE_IPT_GET_CPU_DATA: {
        mx_x86_pt_regs_t regs;
        if (size != sizeof(regs))
            return ERR_INVALID_ARGS;
        uint32_t cpu = MTRACE_IPT_OPTIONS_CPU(options);
        if ((options & ~MTRACE_IPT_OPTIONS_CPU_MASK) != 0)
            return ERR_INVALID_ARGS;
        auto status = x86_ipt_get_cpu_data(cpu, &regs);
        if (status != NO_ERROR)
            return status;
        TRACEF("action %u, cpu %u, ctl 0x%" PRIx64 ", output_base 0x%" PRIx64 "\n",
               action, cpu, regs.ctl, regs.output_base);
        if (arch_copy_to_user(arg, &regs, size) != NO_ERROR)
            return ERR_INVALID_ARGS;
        return NO_ERROR;
    }

    case MTRACE_IPT_CPU_MODE_ALLOC:
        if (options != 0 || size != 0)
            return ERR_INVALID_ARGS;
        return x86_ipt_cpu_mode_alloc();
    case MTRACE_IPT_CPU_MODE_START:
        if (options != 0 || size != 0)
            return ERR_INVALID_ARGS;
        return x86_ipt_cpu_mode_start();
    case MTRACE_IPT_CPU_MODE_STOP:
        if (options != 0 || size != 0)
            return ERR_INVALID_ARGS;
        return x86_ipt_cpu_mode_stop();
    case MTRACE_IPT_CPU_MODE_FREE:
        if (options != 0 || size != 0)
            return ERR_INVALID_ARGS;
        return x86_ipt_cpu_mode_free();

    default:
        return ERR_INVALID_ARGS;
    }
}

#endif
