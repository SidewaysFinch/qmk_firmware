/* Copyright 2021 Jay Greco
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once


/* Used to set host for remote KB if VUSB detect doesn't work. */
// #define KEYBOARD_HOST // Force host mode
// #define KEYBOARD_REMOTE // Force remote mode

// Workaround for freezing after MacOS sleep
// #define NO_USB_STARTUP_CHECK

/* key matrix size */
#define MATRIX_ROWS 5
#define MATRIX_COLS 16
#define MATRIX_MUX_COLS 4

/*
 * Keyboard Matrix Assignments
 * The nibble uses a demultiplexer for the cols.
 * to free up more IOs for awesomeness!
 * See matrix.c for more details.
*/
#define MATRIX_ROW_PINS { B1, B3, B2, B6, D4 }
#define MATRIX_COL_MUX_PINS { F4, F5, F6, F7 }
#define MATRIX_COL_PINS { }

/* Optional SMT LED pins */
#define RGB_DI_PIN E6
#define RGBLIGHT_LED_COUNT 10
#define RGBLIGHT_EFFECT_BREATHING
#define RGBLIGHT_SLEEP

#define RGBLIGHT_DEFAULT_MODE RGBLIGHT_MODE_STATIC_LIGHT
#define RGBLIGHT_DEFAULT_HUE 212 // Sets the default hue value, if none has been set
#define RGBLIGHT_DEFAULT_SAT 62 // Sets the default saturation value, if none has been set
#define RGBLIGHT_DEFAULT_VAL 153 // Sets the default brightness value, if none has been set

/* Optional encoder pins */
#define ENCODERS_PAD_A { B5 }
#define ENCODERS_PAD_B { B4 }

/* Leader key defines. */
#define LEADER_NO_TIMEOUT       /* Leader key itself won't time out. Timing resumes for each key after. */
#define LEADER_PER_KEY_TIMING   /* Each key resets the timer. */
#define LEADER_TIMEOUT 250      /* Timeout in ms. */
