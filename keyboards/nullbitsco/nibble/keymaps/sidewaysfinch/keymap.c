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
#include QMK_KEYBOARD_H
#include "animation_frames.h"

/* Keybind for Discord mutes. */
#define KC_DISC_MUTE    KC_F23
#define KC_SNAP_UP      KC_W
#define KC_SNAP_DOWN    KC_S
#define KC_SNAP_LEFT    KC_A
#define KC_SNAP_RIGHT   KC_D

/* Layer Names. */
enum layer_names {
    _BASE,
    _FN,
    _MOUSEKEYS,
    _GAMING,
};

enum custom_keycodes {
    PROG = SAFE_RANGE,
    DISC_MUTE,
    SNAP_UP,
    SNAP_DOWN,
    SNAP_LEFT,
    SNAP_RIGHT,
};

typedef struct snaptapPair
{
    int keycodeA;
    bool statusA;
    int keycodeB;
    bool statusB;
    bool statusBlock;
} snaptapPair;


snaptapPair snapDict[2] = {
    {KC_SNAP_UP, false, KC_SNAP_DOWN, false, false},
    {KC_SNAP_LEFT, false, KC_SNAP_RIGHT, false, false}
};

/* Macro variables. */
bool muted = false;                 /* Discord muted. */
int snapStatus[4] = {0, 0, 0, 0};   /* Up, Down, Left, Right. */
bool snapUDblock = false;           /* Snap Up/Down pair block. */
bool snapLRblock = false;           /* Snap Left/Right pair block. */

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

/* Keymap Layer definition. */
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* Default layer. */
    [_BASE] = LAYOUT_all(
                  KC_GRAVE,   KC_1,       KC_2,       KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,      KC_0,    KC_MINS, KC_EQL,    KC_BSPC, QK_LEAD,
      DISC_MUTE,  KC_TAB,     KC_Q,       KC_W,       KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,      KC_P,    KC_LBRC, KC_RBRC,   KC_BSLS, KC_DEL,
      QK_LEAD,    KC_ESC,     KC_A,       KC_S,       KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,      KC_SCLN, KC_QUOT,            KC_ENT,  KC_HOME,
      KC_F2,      KC_LSFT,    KC_NUBS,    KC_Z,       KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM,   KC_DOT,  KC_SLSH, KC_RSFT,   KC_UP,   KC_END,
      KC_F1,      KC_LCTL,    KC_LGUI,    KC_LALT,                               KC_SPC,                    MO(_FN), KC_RALT, KC_RCTL, KC_LEFT,   KC_DOWN, KC_RGHT
    ),

    /* FN Layer. */
    [_FN] = LAYOUT_all(
                QK_BOOT,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_INS,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, KC_PGUP,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_PGDN,
      _______,  _______, _______, _______,                            _______,                   _______, _______, _______, KC_MPRV, KC_MPLY, KC_MNXT
    ),

    /* Mousekeys layer. Leader key + KC_2. */
    [_MOUSEKEYS] = LAYOUT_all(
                KC_ESC, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______,  _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______,
      _______,  KC_CAPS, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,           _______, _______,
      _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_BTN1,  KC_MS_U, KC_BTN2,
      _______,  _______, _______, _______,                            _______,                   _______, _______, _______, KC_MS_L,  KC_MS_D, KC_MS_R
    ),

    /* Regular modifiers layer. Leader key + KC_1. */
    [_GAMING] = LAYOUT_all(
                _______, _______,   _______,    _______,    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  _______, _______,   SNAP_UP,    _______,    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  _______, SNAP_LEFT, SNAP_DOWN,  SNAP_RIGHT, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______,
      _______,  _______, _______,   _______,    _______,    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______,  _______, _______,   _______,                                  _______,                   _______, _______, _______, _______, _______, _______
    ),

};

/*!
 * @brief   Handle encoder behaviour.
 */
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

void snapTapStatus(int keyCode, bool enable) {
    // int statusIndex = 0;

    // switch (keyCode) {
    //     case KC_SNAP_UP:
    //         statusIndex = 0;
    //         break;
    //     case KC_SNAP_DOWN:
    //         statusIndex = 1;
    //         break;
    //     case KC_SNAP_LEFT:
    //         statusIndex = 2;
    //         break;
    //     case KC_SNAP_RIGHT:
    //         statusIndex = 3;
    //         break;
    //     default:
    //         break;
    // }

    /* Set snap status and un/press the key. */
    if (enable) {
        // snapStatus[statusIndex] = true;
        register_code(keyCode);
    }
    else {
        // snapStatus[statusIndex] = false;
        unregister_code(keyCode);
    }
}

void handleSnapTap(int pairNo, bool keyA, keyrecord_t *record) {
    int*  keycodeX;
    bool* statusX;
    int*  keycodeY;
    bool* statusY;
    bool* statusBlock;

    /* Determine which of the pair is being handled here. */
    if (keyA == true) {
        keycodeX = &snapDict[pairNo].keycodeA;
        statusX = &snapDict[pairNo].statusA;
        keycodeY = &snapDict[pairNo].keycodeB;
        statusY = &snapDict[pairNo].statusB;
        statusBlock = &snapDict[pairNo].statusBlock;
    }
    else {
        keycodeX = &snapDict[pairNo].keycodeB;
        statusX = &snapDict[pairNo].statusB;
        keycodeY = &snapDict[pairNo].keycodeA;
        statusY = &snapDict[pairNo].statusA;
        statusBlock = &snapDict[pairNo].statusBlock;
    }

    /* Determine whether keyX was pressed. */
    if (record->event.pressed) {
        /* Set keyX as pressed. */
        (*statusX) = true;

        /* Do nothing if blocked. */
        if ((*statusBlock) == true) {
            return;
        }

        /* If keyY already pressed, unsend and block keyY. */
        if ((*statusY) == true) {
            unregister_code((*keycodeY));
            (*statusBlock) = true;
        }

        /* If not blocked, send keyX */
        register_code((*keycodeX));
    }
    else {
        /* Set keyX as unpressed. */
        (*statusX) = false;

        /* If keyY still pressed from our blocked state, re-send keyY. */
        if ((*statusBlock) == true && (*statusY) == true) {
            register_code((*keycodeY));
        }

        /* Unpress keyX, and unblock the pair. */
        unregister_code((*keycodeX));
        (*statusBlock) = false;
    }
}

/*!
 * @brief   Process layer related RGB.
 */
void process_rgb_layers(void) {
    /* Set Base Layer RGB. */
    rgblight_sethsv_noeeprom(RGBLIGHT_DEFAULT_HUE, RGBLIGHT_DEFAULT_SAT, RGBLIGHT_DEFAULT_VAL);

    /* Set mute LED. */
    if (muted) {
        rgblight_sethsv_range(HSV_RED, 0, 1);
    }

    /* Set controller LEDs. */
    if (layer_state_is(_MOUSEKEYS)) {
        rgblight_sethsv_range(HSV_ORANGE, 4, 6);
    }

    /* Set modifier LEDs. */
    if (layer_state_is(_GAMING)) {
        rgblight_sethsv_range(HSV_PURPLE, 6, 9);
    }
}

/*!
 * @brief   Init RGB Post boot.
 */
void keyboard_post_init_user(void) {
    process_rgb_layers();
}

/*!
 * @brief   Handle any code before key event.
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

    /* Handle other keycodes. */
    switch(keycode) {
        /* Jump to bootloader. */
        case PROG:
          if (record->event.pressed) {
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
            muted = !muted;
            process_rgb_layers();
          }
        break;

        case SNAP_UP:
            handleSnapTap(0,true,record);
        break;

        case SNAP_DOWN:
            handleSnapTap(0,false,record);
        break;

        case SNAP_LEFT:
            handleSnapTap(1,true,record);
        break;

        case SNAP_RIGHT:
            handleSnapTap(1,false,record);
        break;

        default:
        break;

    }

    return true;
}

/*!
 * @brief   Run code during every matrix scan.
 */
void matrix_scan_user(void) { }

/*!
 * @brief   Run code when leader sequence starts.
 */
void leader_start_user(void) {
    rgblight_mode_noeeprom(RGBLIGHT_MODE_BREATHING + 3);
}

/*!
 * @brief   Run code when leader sequence ends.
 */
void leader_end_user(void) {
        /* Toggle swap Gaming layer. */
        if (leader_sequence_one_key(KC_1)) {
            layer_invert(_GAMING);
        }
        /* Toggle Mousekeys controller layer. */
        else if (leader_sequence_one_key(KC_2)) {
            layer_invert(_MOUSEKEYS);
        }
        /* Touch capslock. */
        else if (leader_sequence_one_key(KC_ESC)) {
            tap_code(KC_CAPS);
        }
        /* Alt+F4. */
        else if (leader_sequence_two_keys(KC_W, KC_Q)) {
            register_code(KC_LALT);
            register_code(KC_F4);
            unregister_code(KC_LALT);
            unregister_code(KC_F4);
        }
        /* PR note [Nit]. */
        else if (leader_sequence_two_keys(KC_R, KC_N)) {
            SEND_STRING("[Nit] ");
        }
        /* PR note [Question]. */
        else if (leader_sequence_two_keys(KC_R, KC_Q)) {
            SEND_STRING("[Question] ");
        }
        /* PR note [Suggestion]. */
        else if (leader_sequence_two_keys(KC_R, KC_S)) {
            SEND_STRING("[Suggestion] ");
        }
        /* PR note [Minor]. */
        else if (leader_sequence_two_keys(KC_R, KC_M)) {
            SEND_STRING("[Minor] ");
        }
        /* PR note [Major]. */
        else if (leader_sequence_three_keys(KC_R, KC_M, KC_A)) {
            SEND_STRING("[Major] ");
        }

        /* Turn off light effect. */
        rgblight_mode_noeeprom(RGBLIGHT_MODE_STATIC_LIGHT);
        process_rgb_layers();
}
