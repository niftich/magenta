// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008-2014 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef __ERR_H
#define __ERR_H

#ifndef ASSEMBLY
#include <sys/types.h> // for status_t
#endif

#include <magenta/errors.h>

// TODO: This is used primarily by class drivers which are obsolete.
// Re-examine when those are removed.
#define ERR_NOT_CONFIGURED (-501)

// MOVE to kernel internal used for thread teardown
#define ERR_INTERRUPTED (-502)

#endif
