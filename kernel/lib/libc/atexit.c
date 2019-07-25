// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2008-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT


/* nulled out atexit. static object constructors call this */
int atexit(void (*func)(void));
int atexit(void (*func)(void))
{
    return 0;
}

