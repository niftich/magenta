// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <linenoise/linenoise.h>

void tab_complete(const char* line, linenoiseCompletions* completions);
