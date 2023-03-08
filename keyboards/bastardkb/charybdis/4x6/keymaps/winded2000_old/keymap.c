/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
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
#include "features/sentence_case.h"

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#    include "timer.h"
#endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

enum charybdis_keymap_layers {   // define layer names
    LAYER_BASE = 0,
    LAYER_LOWER,
    LAYER_RAISE,
    LAYER_POINTER,
};
/** \brief Automatically enable sniping-mode on the pointer layer. */
#define CHARYBDIS_AUTO_SNIPING_ON_LAYER LAYER_POINTER

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
static uint16_t auto_pointer_layer_timer = 0;

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS 1000
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD 8
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#endif     // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#define LOWER MO(LAYER_LOWER)
#define RAISE MO(LAYER_RAISE)
//#define TL_LOWR QK_TRI_LAYER_LOWER
//#define TL_UPPR QK_TRI_LAYER_UPPER
#define PT_Z LT(LAYER_POINTER, KC_Z)
#define PT_SLSH LT(LAYER_POINTER, KC_SLSH)
#define HRM_U LCTL_T(KC_U)
#define HRM_E LSFT_T(KC_E)
#define HRM_Q LALT_T(KC_Q)
#define HRM_SCL LGUI_T(KC_SCLN)
#define HRM_H RCTL_T(KC_H)
#define HRM_T RSFT_T(KC_T)
#define HRM_V LALT_T(KC_V)
#define HRM_Z RGUI_T(KC_Z)

enum custom_keycodes {   // define macro names
    MC_NAME = SAFE_RANGE,
    MC_LAST,
    MC_ADDY,
    MC_EMAL,
    MC_SENT,
    MC_PSTV,
    MC_DEG,
    MC_PSMS,
    MC_PARN,
    MC_BRKT,
    MC_BRCE,
    MC_SD,
    MC_REBT,
};

bool process_record_user(uint16_t keycode, keyrecord_t* record) {   // define macro functionality
    if (!process_sentence_case(keycode, record)) { return false; }

    switch (keycode) {
    case MC_NAME:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("Thaddeus");
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_LAST:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("Kiedaisch");
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_ADDY:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("1127 Andrews Peak Dr.");
		} else {
			// when keycode MC_NAME is released
		}
		break;

	case MC_EMAL:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("winded@kiedaisch.us");
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_SENT:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING(". ");
			add_oneshot_mods(MOD_BIT(KC_LSFT));
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_DEG:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING(SS_LALT(SS_TAP(X_KP_2) SS_TAP(X_KP_4) SS_TAP(X_KP_8)));
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_PSMS:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING(SS_LALT(SS_TAP(X_KP_2) SS_TAP(X_KP_4) SS_TAP(X_KP_1)));
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_PARN:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("()");
			tap_code(KC_LEFT);
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_BRKT:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("[]");
			tap_code(KC_LEFT);
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_BRCE:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING("{}");
			tap_code(KC_LEFT);
		} else {
			// when keycode MC_NAME is released
		}
		break;
	}
	return true;
};
// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
        KC_DEL,    KC_1,    KC_2,    KC_3,    KC_4,    KC_5,       KC_6,    KC_7,    KC_8,    KC_9,    KC_0, KC_MINS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
        KC_TAB, KC_QUOT, KC_COMM,  KC_DOT,    KC_P,    KC_Y,       KC_F,    KC_G,    KC_C,    KC_R,    KC_L, KC_BSPC,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
        KC_ESC,    KC_A,    KC_O,   HRM_E,   HRM_U,    KC_I,       KC_D,   HRM_H,   HRM_T,    KC_N,    KC_S, KC_SLSH,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
 OSM(MOD_LSFT), HRM_SCL,   HRM_Q,    KC_J,    KC_K,    KC_X,       KC_B,    KC_M,    KC_W,   HRM_V,   HRM_Z,  KC_ENT,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                   KC_LGUI, KC_SPC, TL_LOWR,    TL_UPPR,  KC_ENT,
                                           KC_LALT, KC_BSPC,     KC_DEL
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_LOWER] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
       KC_TILD, KC_EXLM,   KC_AT, KC_HASH,  KC_DLR, KC_PERC,    KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_UNDS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       RGB_MOD, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,    KC_LBRC,    KC_7,    KC_8,    KC_9, KC_RBRC, XXXXXXX,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       RGB_TOG, KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, XXXXXXX,    KC_PPLS,    KC_4,    KC_5,    KC_6, KC_PMNS, KC_PEQL,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      RGB_RMOD, XXXXXXX, XXXXXXX, XXXXXXX, EE_CLR,  QK_BOOT,    KC_PAST,    KC_1,    KC_2,    KC_3, KC_PSLS, KC_PDOT,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  XXXXXXX, XXXXXXX, _______,    _______, _______,
                                           XXXXXXX, XXXXXXX,      KC_P0
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_RAISE] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
        KC_F12,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,      KC_F6,   KC_F7,   KC_F8,   KC_F9,  KC_F10,  KC_F11,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       KC_MNXT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_VOLU,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       KC_MPLY, KC_LEFT,   KC_UP, KC_DOWN, KC_RGHT, XXXXXXX,    XXXXXXX, KC_RSFT, KC_RCTL, KC_RALT, KC_RGUI, KC_MUTE,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       KC_MPRV, KC_HOME, KC_PGUP, KC_PGDN,  KC_END, XXXXXXX,    QK_BOOT, EE_CLR,  XXXXXXX, XXXXXXX, XXXXXXX, KC_VOLD,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, XXXXXXX,
                                           _______, _______,    XXXXXXX
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),
  
  [LAYER_POINTER] = LAYOUT_charybdis_4x6(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
       XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, DPI_MOD, S_D_MOD,    S_D_MOD, DPI_MOD, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       XXXXXXX, KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, XXXXXXX,    XXXXXXX, KC_RSFT, KC_RCTL, KC_RALT, KC_RGUI, XXXXXXX,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
       XXXXXXX, _______, DRGSCRL, SNIPING, _______, QK_BOOT,    QK_BOOT, _______, SNIPING, DRGSCRL, _______, XXXXXXX,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  KC_BTN2, KC_BTN1, KC_BTN3,    KC_BTN3, KC_BTN1,
                                           XXXXXXX, KC_BTN2,    KC_BTN2
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),
};

#ifdef POINTING_DEVICE_ENABLE
#    ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (abs(mouse_report.x) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD || abs(mouse_report.y) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD) {
        if (auto_pointer_layer_timer == 0) {
            layer_on(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
            rgb_matrix_mode_noeeprom(RGB_MATRIX_NONE);
            rgb_matrix_sethsv_noeeprom(HSV_GREEN);
#        endif // RGB_MATRIX_ENABLE
        }
        auto_pointer_layer_timer = timer_read();
    }
    return mouse_report;
}

void matrix_scan_user(void) {
    if (auto_pointer_layer_timer != 0 && TIMER_DIFF_16(timer_read(), auto_pointer_layer_timer) >= CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS) {
        auto_pointer_layer_timer = 0;
        layer_off(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
        rgb_matrix_mode_noeeprom(RGB_MATRIX_STARTUP_MODE);
#        endif // RGB_MATRIX_ENABLE
    }
}
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#    ifdef CHARYBDIS_AUTO_SNIPING_ON_LAYER
layer_state_t layer_state_set_user(layer_state_t state) {
    charybdis_set_pointer_sniping_enabled(layer_state_cmp(state, CHARYBDIS_AUTO_SNIPING_ON_LAYER));
    return state;
}
#    endif // CHARYBDIS_AUTO_SNIPING_ON_LAYER
#endif     // POINTING_DEVICE_ENABLE

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);
#endif

void shutdown_user(void) {
#ifdef RGBLIGHT_ENABLE
    rgblight_enable_noeeprom();
    rgblight_mode_noeeprom(1);
    rgblight_setrgb_red();
#endif // RGBLIGHT_ENABLE
#ifdef RGB_MATRIX_ENABLE
    rgb_matrix_set_color_all(RGB_RED);
    rgb_matrix_update_pwm_buffers();
#endif // RGB_MATRIX_ENABLE
}

const uint16_t PROGMEM asn_combo[] = {KC_A, KC_S, KC_N, COMBO_END};
const uint16_t PROGMEM ask_combo[] = {KC_A, KC_S, KC_K, COMBO_END};
const uint16_t PROGMEM asf_combo[] = {KC_A, KC_S, KC_F, COMBO_END};
const uint16_t PROGMEM asw_combo[] = {KC_A, KC_S, KC_W, COMBO_END};
const uint16_t PROGMEM test_combo5[] = {KC_QUOT, KC_COMM, COMBO_END};
const uint16_t PROGMEM test_combo6[] = {KC_MINS, KC_EQL, COMBO_END};
combo_t key_combos[COMBO_COUNT] = {
    COMBO(asn_combo, MC_NAME),
    COMBO(ask_combo, MC_LAST),
    COMBO(asf_combo, MC_ADDY),
    COMBO(asw_combo, MC_EMAL),
    COMBO(test_combo5, MC_DEG),
    COMBO(test_combo6, MC_PSMS),  // keycodes with modifiers are possible too!
};
// clang-format on
