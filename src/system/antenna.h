/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_ANTENNA
#define SLIMENRF_ANTENNA

#include <stdbool.h>
#include <stdint.h>

// Initialize antenna diversity control
int antenna_init(void);

// Switch to antenna 0 (ANT-SET = LOW)
void antenna_select_0(void);

// Switch to antenna 1 (ANT-SET = HIGH)
void antenna_select_1(void);

// Toggle antenna (switch to the other antenna)
void antenna_toggle(void);

// Get current antenna selection (0 or 1)
uint8_t antenna_get_current(void);

// Periodic antenna switching (call this periodically for diversity)
void antenna_periodic_switch(void);

#endif

