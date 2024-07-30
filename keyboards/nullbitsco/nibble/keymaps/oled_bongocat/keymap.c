/* Copyright 2021 Jonathan Law, Jay Greco
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
 *
 * Original: j-inc's kyria keymap
 */
#include "../../nibble.h" //QMK_KEYBOARD_H
#include "animation_frames.h"

LEADER_EXTERNS();

/* Keybind for Discord mutes. */
#define KC_DISC_MUTE KC_F23

/* Default RGB Color. */
#define RGBLIGHT_DEFAULT_HUE 212
#define RGBLIGHT_DEFAULT_SAT 62
#define RGBLIGHT_DEFAULT_VAL 153

/* Controller Color. */
#define RGB_CONTROLLER_R 255
#define RGB_CONTROLLER_G 95
#define RGB_CONTROLLER_B 1

/* VIA layer names. */
enum layer_names {
    _BASE,
    _VIA1,
    _VIA2,
    _VIA3
};

/* Custom Keycodes. */
enum custom_keycodes {
    PROG = SAFE_RANGE,
    DISC_MUTE,
};

/* Macro variables. */
bool muted = false;         /* Discord muted. */
bool controllerMode = false;

/* Keymap Layer definition. */
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* Default layer. */
    [_BASE] = LAYOUT_all(
                  KC_GRAVE,   KC_1,       KC_2,       KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,      KC_0,    KC_MINS, KC_EQL,    KC_BSPC, KC_LEAD,
      DISC_MUTE,  KC_TAB,     KC_Q,       KC_W,       KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,      KC_P,    KC_LBRC, KC_RBRC,   KC_BSLS, KC_DEL,
      KC_LEAD,    KC_ESC,     KC_A,       KC_S,       KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,      KC_SCLN, KC_QUOT,            KC_ENT,  KC_HOME,
      KC_F2,      KC_LSFT,    KC_NUBS,    KC_Z,       KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM,   KC_DOT,  KC_SLSH, KC_RSFT,   KC_UP,   KC_END,
      KC_F1,      KC_LCTL,    KC_LGUI,    KC_LALT,                               KC_SPC,                    MO(_VIA1), KC_RALT, KC_RCTL, KC_LEFT,   KC_DOWN, KC_RGHT
    ),

    /* FN Layer. */
    [_VIA1] = LAYOUT_all(
                RESET,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_INS,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, KC_PGUP,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_PGDN,
      _______,  _______, _______, _______,                            _______,                   _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT
    ),

    /* Controller layer. Leader key + KC_2. */
    [_VIA2] = LAYOUT_all(
                KC_ESC, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,  _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______,
      _______,  KC_CAPS, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______,
      _______,  _______, _______, _______,                            _______,                   _______, _______, _______, _______,  _______, _______
    ),

    /* Regular modifiers layer. Leader key + KC_1. */
    [_VIA3] = LAYOUT_all(
                KC_ESC, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  KC_CAPS, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  _______, _______, _______,                            _______,                   _______, _______, _______, _______, _______, _______
    ),

};

/* Handle encoder behaviour. */
bool encoder_update_user(uint8_t index, bool clockwise) {
    /* Use encoder as a volume or scroll wheel depending on FN layer. */
    if (clockwise) {
        tap_code(KC_WH_D);
    }
    else {
        tap_code(KC_WH_U);
    }
    return true;
}

/* ----------------------
 *    OLED CODE BEGIN
 * ----------------------
 */
#ifdef OLED_ENABLE
#define IDLE_FRAME_DURATION 200 // Idle animation iteration rate in ms

oled_rotation_t oled_init_user(oled_rotation_t rotation) { return OLED_ROTATION_90; }

uint32_t anim_timer         = 0;
uint32_t anim_sleep         = 0;
uint8_t  current_idle_frame = 0;

bool tap_anim        = false;
bool tap_anim_toggle = false;


// Decompress and write a precompressed bitmap frame to the OLED.
// Documentation and python compression script available at:
// https://github.com/nullbitsco/squeez-o
#ifdef USE_OLED_BITMAP_COMPRESSION
static void oled_write_compressed_P(const char* input_block_map, const char* input_block_list) {
    uint16_t block_index = 0;
    for (uint16_t i=0; i<NUM_OLED_BYTES; i++) {
        uint8_t bit = i%8;
        uint8_t map_index = i/8;
        uint8_t _block_map = (uint8_t)pgm_read_byte_near(input_block_map + map_index);
        uint8_t nonzero_byte = (_block_map & (1 << bit));
        if (nonzero_byte) {
            const char data = (const char)pgm_read_byte_near(input_block_list + block_index++);
            oled_write_raw_byte(data, i);
        } else {
            const char data = (const char)0x00;
            oled_write_raw_byte(data, i);
        }
    }
}
#endif

static void render_anim(void) {
    // Idle animation
    void animation_phase(void) {
        if (!tap_anim) {
            current_idle_frame = (current_idle_frame + 1) % NUM_IDLE_FRAMES;
            uint8_t idx = abs((NUM_IDLE_FRAMES - 1) - current_idle_frame);
            #ifdef USE_OLED_BITMAP_COMPRESSION
            oled_write_compressed_P(idle_block_map[idx], idle_frames[idx]);
            #else
            oled_write_raw_P(idle_frames[idx], NUM_OLED_BYTES);
            #endif
        }
    }

    // Idle behaviour
    if (get_current_wpm() != 000) {  // prevent sleep
        oled_on();
        if (timer_elapsed32(anim_timer) > IDLE_FRAME_DURATION) {
            anim_timer = timer_read32();
            animation_phase();
        }
        anim_sleep = timer_read32();
    } else {  // Turn off screen when timer threshold elapsed or reset time since last input
        if (timer_elapsed32(anim_sleep) > OLED_TIMEOUT) {
            oled_off();
        } else {
            if (timer_elapsed32(anim_timer) > IDLE_FRAME_DURATION) {
                anim_timer = timer_read32();
                animation_phase();
            }
        }
    }
}

bool oled_task_user(void) {
    render_anim();
    oled_set_cursor(0, 14);

    uint8_t n = get_current_wpm();
    char wpm_counter[6];
    wpm_counter[5] = '\0';
    wpm_counter[4] = '0' + n % 10;
    wpm_counter[3] = '0' + (n /= 10) % 10;
    wpm_counter[2] = '0' + n / 10 ;
    wpm_counter[1] = '0';
    wpm_counter[0] = '>';
    oled_write_ln(wpm_counter, false);

    return false;
}
#endif
/* ----------------------
 *     OLED CODE END
 * ----------------------
 */

bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    /* Animate OLED Tap. */
    #ifdef OLED_ENABLE
    // Check if non-mod
    if ((keycode >= KC_A && keycode <= KC_0) || (keycode >= KC_TAB && keycode <= KC_SLASH)) {
        if (record->event.pressed) {
            // Display tap frames
            tap_anim_toggle = !tap_anim_toggle;
            #ifdef USE_OLED_BITMAP_COMPRESSION
            oled_write_compressed_P(tap_block_map[tap_anim_toggle], tap_frames[tap_anim_toggle]);
            #else
            oled_write_raw_P(tap_frames[tap_anim_toggle], NUM_OLED_BYTES);
            #endif
        }
    }
    #endif

    /* Handle other keyboards. */
    switch(keycode) {
        /* Jump to bootloader. */
        case PROG:
          if (record->event.pressed) {
            rgblight_disable_noeeprom();
            #ifdef OLED_ENABLE
            oled_off();
            #endif
            bootloader_jump();
          }
        break;

        /* Mute on discord. */
        case DISC_MUTE:
          if (record->event.pressed) {
            tap_code(KC_DISC_MUTE);
            if (!rgblight_is_enabled()) break;

            /* Toggle red light. */
            if (muted) {
              rgblight_enable_noeeprom();
            } else {
              rgblight_timer_disable();
              uint8_t val = rgblight_get_val();
              rgblight_sethsv_range(255, 255, val, 0, 1);
            }
            muted = !muted;
          }
        break;

        default:
        break;
    }

    return true;
}

void matrix_scan_user(void) {
    /* Handle leader key operations. */
    LEADER_DICTIONARY() {
        leading = false;
        leader_end();

        /* Toggle swap grave/escape layer. */
        SEQ_ONE_KEY(KC_1) {
            layer_invert(_VIA3);
        }
        /* Toggle GCN controller layer. */
        SEQ_ONE_KEY(KC_2) {
            layer_invert(_VIA2);
            controllerMode ^= 1;

            /* Set controller light to a different layer. */
            if (controllerMode) {
                rgblight_setrgb_range(RGB_CONTROLLER_R, RGB_CONTROLLER_G, RGB_CONTROLLER_B, 0, RGBLED_NUM);
            }
            else {
                rgblight_sethsv_range(RGBLIGHT_DEFAULT_HUE, RGBLIGHT_DEFAULT_SAT, RGBLIGHT_DEFAULT_VAL, 0, RGBLED_NUM);
            }
        }
        /* Touch capslock. */
        SEQ_ONE_KEY(KC_ESC) {
            tap_code(KC_CAPS);
        }
        /* Alt+F4. */
        SEQ_TWO_KEYS(KC_W, KC_Q) {
            register_code(KC_LALT);
            register_code(KC_F4);
            unregister_code(KC_LALT);
            unregister_code(KC_F4);
        }
        /* PR note [Nit]. */
        SEQ_TWO_KEYS(KC_R, KC_N) {
            SEND_STRING("[Nit] ");
        }
        /* PR note [Question]. */
        SEQ_TWO_KEYS(KC_R, KC_Q) {
            SEND_STRING("[Question] ");
        }
        /* PR note [Suggestion]. */
        SEQ_TWO_KEYS(KC_R, KC_S) {
            SEND_STRING("[Suggestion] ");
        }
        /* PR note [Minor]. */
        SEQ_TWO_KEYS(KC_R, KC_M) {
            SEND_STRING("[Minor] ");
        }
        /* PR note [Major]. */
        SEQ_THREE_KEYS(KC_R, KC_M, KC_A) {
            SEND_STRING("[Major] ");
        }
    }
}

void leader_start(void) {
    rgblight_mode(RGBLIGHT_MODE_BREATHING + 3);
}

void leader_end(void) {
    rgblight_mode(RGBLIGHT_MODE_STATIC_LIGHT);
}