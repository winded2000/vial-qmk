#include QMK_KEYBOARD_H
#include "features/sentence_case.h"

enum layers {
  _DVORAK,
  _LOWER,
  _RAISE,
  _ADJUST
};

#define LOWER TT(_LOWER)
#define RAISE TT(_RAISE)
#define HRM_U LCTL_T(KC_U)
#define HRM_E LSFT_T(KC_E)
#define HRM_Q LALT_T(KC_Q)
#define HRM_SCL LGUI_T(KC_SCLN)
#define HRM_H RCTL_T(KC_H)
#define HRM_T RSFT_T(KC_T)
#define HRM_V LALT_T(KC_V)
#define HRM_Z RGUI_T(KC_Z)

enum custom_keycodes {   // define macro names
    MC_NAME = USER00,
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
    MC_PASTE_VALU,
    REPEAT,          // For use of REPEAT key if desired
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
			SEND_STRING(SS_DOWN(X_LALT) SS_TAP(X_KP_2) SS_TAP(X_KP_4) SS_TAP(X_KP_8) SS_UP(X_LALT));
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_PSMS:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
			SEND_STRING(SS_DOWN(X_LALT) SS_TAP(X_KP_2) SS_TAP(X_KP_4) SS_TAP(X_KP_1) SS_UP(X_LALT));
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

    case MC_SD:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
            SEND_STRING(SS_LGUI("x") SS_DELAY(200) "u" SS_DELAY(200) "u" SS_DELAY(200) "u"); 
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_REBT:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
            SEND_STRING(SS_LGUI("x") SS_DELAY(200) "u" SS_DELAY(200) "r");
		} else {
			// when keycode MC_NAME is released
		}
		break;

    case MC_PASTE_VALU:
        if (record->event.pressed) {
			// when keycode MC_NAME is pressed
            SEND_STRING(SS_LCTL(SS_LALT("v")) SS_DELAY(200) "v");
            tap_code(KC_ENT);
		} else {
			// when keycode MC_NAME is released
		}
		break;
	}
	return true;
};

// Tap Dance declarations
enum {
    TD_CAPS,
    TD_VDN,
    TD_MDR,
    TD_PST,
    TD_HOM,
    TD_END,
    TD_PSN,
    TD_PAR,
    TD_BKT,
    TD_BRC,
};

// Tap Dance definitions
tap_dance_action_t tap_dance_actions[] = {    // Tap once for Escape, twice for Caps Lock
    [TD_CAPS] = ACTION_TAP_DANCE_DOUBLE(OSM(MOD_LSFT), CW_TOGG),
    [TD_VDN] = ACTION_TAP_DANCE_DOUBLE(KC_VOLD, KC_MUTE),
    [TD_MDR] = ACTION_TAP_DANCE_DOUBLE(KC_MPLY, KC_MNXT),
    [TD_PST] = ACTION_TAP_DANCE_DOUBLE(LCTL(KC_V), MC_PASTE_VALU),
    [TD_HOM] = ACTION_TAP_DANCE_DOUBLE(KC_HOME, LCTL(KC_HOME)),
    [TD_END] = ACTION_TAP_DANCE_DOUBLE(KC_END, LCTL(KC_END)),
    [TD_PSN] = ACTION_TAP_DANCE_DOUBLE(KC_PSCR, LCTL(KC_PSCR)),
    [TD_PAR] = ACTION_TAP_DANCE_DOUBLE(KC_LPRN, MC_PARN),
    [TD_BKT] = ACTION_TAP_DANCE_DOUBLE(KC_LBRC, MC_BRKT),
    [TD_BRC] = ACTION_TAP_DANCE_DOUBLE(KC_LCBR, MC_BRCE),
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Dvorak
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   '  |   ,  |   .  |   P  |   Y  |   F  |   G  |   C  |   R  |   L  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   A  |   O  |  *E  |  *U  |   I  |   D  |  *H  |  *T  |   N  |   S  |  /   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|  *;  |  *Q  |   J  |   K  |   X  |   B  |   M  |   W  |  *V  |  *Z  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | BLTog| Ctrl | GUI  | Alt  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
 * `-----------------------------------------------------------------------------------'
 */
[_DVORAK] = LAYOUT_planck_mit(
    KC_TAB, KC_QUOT, KC_COMM,  KC_DOT,    KC_P,    KC_Y,    KC_F,    KC_G,    KC_C,    KC_R,    KC_L, KC_BSPC,
    KC_ESC,    KC_A,    KC_O,   HRM_E,   HRM_U,    KC_I,    KC_D,   HRM_H,   HRM_T,    KC_N,    KC_S, KC_SLSH,
 OSM(MOD_LSFT), HRM_SCL, HRM_Q,  KC_J,    KC_K,    KC_X,    KC_B,    KC_M,    KC_W,   HRM_V,   HRM_Z,  SC_SENT,
    KC_DEL, KC_LCTL, KC_LGUI, KC_LALT, LOWER,  LSFT_T(KC_SPACE), RAISE, KC_MPRV, TD(TD_VDN), KC_VOLU, TD(TD_MDR)
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   _  |   +  |   {  |   }  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |ISO ~ |ISO | | Home | End  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      | Next | Vol- | Vol+ | Play |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_mit(
    RGB_MOD,   KC_F1,   KC_F2,   KC_F3,   KC_F4, TD(TD_BRC), KC_RCBR,    KC_7,    KC_8,    KC_9, KC_PPLS, _______,
    RGB_TOG, KC_F5,   KC_F6, LSFT_T(KC_F7), LCTL_T(KC_F8), TD(TD_BKT), KC_RBRC, KC_4, KC_5, KC_6, KC_MINS, _______,
    RGB_RMOD,  KC_F9,  KC_F10,  KC_F11,  KC_F12, TD(TD_PAR), KC_RPRN,    KC_1,    KC_2,    KC_3,  KC_EQL, _______,
    _______, _______, _______, _______, _______, _______, _______,    KC_0,    KC_DOT, _______, _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   -  |   =  |   [  |   ]  |  \   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |ISO # |ISO / |Pg Up |Pg Dn |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      | Next | Vol- | Vol+ | Play |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_mit(
    KC_GRV,  KC_P1,   KC_P2,   KC_P3,   KC_P4,   KC_P5,   KC_P6,   KC_P7,   KC_P8,   KC_P9,   KC_P0,  KC_DEL,
    _______, TD(TD_PSN), LCTL(KC_X), LCTL(KC_C), TD(TD_PST), XXXXXXX, KC_PGUP, TD(TD_HOM), KC_UP, TD(TD_END), _______, KC_BSLS,
    CW_TOGG, KC_WSCH, KC_WHOM, KC_MAIL, KC_MYCM, XXXXXXX,    KC_PGDN, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),

/* Adjust (Lower + Raise)
 *                      v------------------------RGB CONTROL--------------------v
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|Debug | RGB  |RGBMOD| HUE+ | HUE- | SAT+ | SAT- |BRGTH+|BRGTH-|  Del |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_mit(
      RESET, _______, _______, _______, _______, _______, _______, _______, _______, MC_REBT, RGUI(KC_L), MC_SD ,
    _______, _______, RGB_TOG, RGB_VAI, KC_BRIU, _______, _______, KC_BTN1, KC_MS_U,  KC_BTN2, KC_WH_U, _______,
    _______, _______, RGB_MOD, RGB_VAD, KC_BRID, _______, _______, KC_MS_L, KC_MS_D,  KC_MS_R, KC_WH_D, _______,
    _______, _______, _______, _______, _______,   LCA(KC_DEL),    _______, _______,  _______, _______, _______
)

};

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}
const uint16_t PROGMEM asn_combo[] = {KC_A, KC_S, KC_N, COMBO_END};
const uint16_t PROGMEM ask_combo[] = {KC_A, KC_S, KC_K, COMBO_END};
const uint16_t PROGMEM asf_combo[] = {KC_A, KC_S, KC_F, COMBO_END};
const uint16_t PROGMEM asw_combo[] = {KC_A, KC_S, KC_W, COMBO_END};
const uint16_t PROGMEM test_combo5[] = {KC_QUOT, KC_DOT, COMBO_END};
const uint16_t PROGMEM test_combo6[] = {KC_MINS, KC_EQL, COMBO_END};
combo_t key_combos[COMBO_COUNT] = {
    COMBO(asn_combo, MC_NAME),
    COMBO(ask_combo, MC_LAST),
    COMBO(asf_combo, MC_ADDY),
    COMBO(asw_combo, MC_EMAL),
    COMBO(test_combo5, MC_DEG),
    COMBO(test_combo6, MC_PSMS),  // keycodes with modifiers are possible too!
};