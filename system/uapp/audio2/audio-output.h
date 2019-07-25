// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <magenta/types.h>

#include "audio-stream.h"

class AudioSource;

class AudioOutput : public AudioStream {
public:
    mx_status_t Play(AudioSource& source);

private:
    friend class mxtl::unique_ptr<AudioOutput>;
    friend class AudioStream;

    explicit AudioOutput(uint32_t dev_id) : AudioStream(false, dev_id) { }
};

