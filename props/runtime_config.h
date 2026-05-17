// ProffieOS/props/runtime_config.h
#ifndef PROPS_RUNTIME_CONFIG_H
#define PROPS_RUNTIME_CONFIG_H

#include "../common/preset.h"

#ifndef INI_MAX_KEY_LEN
#define INI_MAX_KEY_LEN 32
#endif

#define INI_MAX_PRESETS 20
#define INI_MAX_SLOTS 23
#define INI_MAX_STYLE_NAME_LEN 24
#define INI_MAX_FONT_PATH_LEN 64
#define INI_MAX_TRACK_PATH_LEN 64

#define GESTURE_TWIST_ON      (1 << 0)
#define GESTURE_TWIST_OFF     (1 << 1)
#define GESTURE_STAB_ON       (1 << 2)
#define GESTURE_SWING_ON      (1 << 3)
#define GESTURE_THRUST_ON     (1 << 4)
#define GESTURE_FORCE_PUSH    (1 << 5)
#define GESTURE_MELT          (1 << 6)

#define OFF_MODE_PULSE  0
#define OFF_MODE_RANDOM 1

enum IniAction {
  ACTION_NONE = 0,
  ACTION_ON,
  ACTION_OFF,
  ACTION_BLAST,
  ACTION_CLASH,
  ACTION_LOCKUP,
  ACTION_DRAG,
  ACTION_MELT,
  ACTION_LIGHTNING_BLOCK,
  ACTION_FORCE,
  ACTION_STAB,
  ACTION_COLOR_CHANGE,
  ACTION_NEXT_PRESET,
  ACTION_PREV_PRESET,
  ACTION_VOLUME_UP,
  ACTION_VOLUME_DOWN,
  ACTION_TRACK_PLAYER,
  ACTION_BATTERY_LEVEL,
  ACTION_QUOTE,
  ACTION_ENTER_COLOR_CHANGE,
  ACTION_EXIT_COLOR_CHANGE,
  ACTION_COUNT
};

struct ActionNameEntry {
  const char* name;
  IniAction action;
};

const ActionNameEntry action_name_table[] = {
  {"none", ACTION_NONE},
  {"on", ACTION_ON},
  {"off", ACTION_OFF},
  {"blast", ACTION_BLAST},
  {"clash", ACTION_CLASH},
  {"lockup", ACTION_LOCKUP},
  {"drag", ACTION_DRAG},
  {"melt", ACTION_MELT},
  {"lightning_block", ACTION_LIGHTNING_BLOCK},
  {"force", ACTION_FORCE},
  {"stab", ACTION_STAB},
  {"color_change", ACTION_COLOR_CHANGE},
  {"next_preset", ACTION_NEXT_PRESET},
  {"prev_preset", ACTION_PREV_PRESET},
  {"volume_up", ACTION_VOLUME_UP},
  {"volume_down", ACTION_VOLUME_DOWN},
  {"track_player", ACTION_TRACK_PLAYER},
  {"battery_level", ACTION_BATTERY_LEVEL},
  {"quote", ACTION_QUOTE},
  {"enter_color_change", ACTION_ENTER_COLOR_CHANGE},
  {"exit_color_change", ACTION_EXIT_COLOR_CHANGE},
};

const int ACTION_NAME_COUNT = sizeof(action_name_table) / sizeof(action_name_table[0]);

IniAction LookupAction(const char* name) {
  for (int i = 0; i < ACTION_NAME_COUNT; i++) {
    if (strcasecmp(name, action_name_table[i].name) == 0) {
      return action_name_table[i].action;
    }
  }
  return ACTION_NONE;
}

struct IniPreset {
  char font[INI_MAX_FONT_PATH_LEN];
  char track[INI_MAX_TRACK_PATH_LEN];
  char style_name[INI_MAX_STYLE_NAME_LEN];
  char name[INI_MAX_KEY_LEN];

  char base_color[20];
  char alt_color[20];
  char blast_color[20];
  char clash_color[20];
  char lockup_color[20];
  char drag_color[20];
  char lb_color[20];
  char stab_color[20];
  char swing_color[20];
  char emitter_color[20];
  char preon_color[20];
  char off_color[20];

  uint16_t ignition_time;
  uint16_t retraction_time;

  char accent_style[INI_MAX_STYLE_NAME_LEN];
  uint16_t accent_speed;

  uint16_t flicker_depth;
  uint16_t flicker_speed;
  uint16_t stripe_width;
  uint16_t stripe_speed;
  uint16_t motion_gain;
  uint16_t noise_mix;
  uint16_t core_contrast;
  uint16_t pulse_rate;
  uint16_t pulse_depth;
  uint16_t strobe_freq;
  uint16_t strobe_ms;
  uint16_t drift_rate;
  uint16_t warm_shift;
  uint16_t jitter_amount;
  uint16_t spark_mix;
  uint16_t heat_rand;
  uint16_t fire_cooling;
  uint16_t rainbow_speed;

  uint8_t off_mode;
  uint16_t off_rate_ms;

  void SetDefaults() {
    strcpy(font, "font1");
    track[0] = 0;
    strcpy(style_name, "standard");
    strcpy(name, "Default");

    strcpy(base_color, "0,0,65535");
    strcpy(alt_color, "0,65535,65535");
    strcpy(blast_color, "65535,65535,65535");
    strcpy(clash_color, "65535,65535,65535");
    strcpy(lockup_color, "65535,65535,65535");
    strcpy(drag_color, "65535,20560,0");
    strcpy(lb_color, "0,65535,65535");
    strcpy(stab_color, "65535,65535,65535");
    strcpy(swing_color, "0,0,0");
    strcpy(emitter_color, "0,0,65535");
    strcpy(preon_color, "0,0,65535");
    strcpy(off_color, "0,0,0");

    ignition_time = 300;
    retraction_time = 200;

    accent_style[0] = 0;
    accent_speed = 1000;

    flicker_depth = 12000;
    flicker_speed = 1000;
    stripe_width = 5000;
    stripe_speed = 900;
    motion_gain = 4096;
    noise_mix = 8000;
    core_contrast = 32768;
    pulse_rate = 1200;
    pulse_depth = 9000;
    strobe_freq = 15;
    strobe_ms = 1;
    drift_rate = 600;
    warm_shift = 2000;
    jitter_amount = 1200;
    spark_mix = 5000;
    heat_rand = 4500;
    fire_cooling = 55;
    rainbow_speed = 800;

    off_mode = OFF_MODE_PULSE;
    off_rate_ms = 1200;
  }
};

struct IniGlobalConfig {
  uint8_t volume;
  uint8_t clash_threshold;
  uint8_t gesture_flags;
  uint8_t num_buttons;
  char button_profile[INI_MAX_KEY_LEN];

  void SetDefaults() {
    volume = 80;
    clash_threshold = 8;
    gesture_flags = GESTURE_TWIST_ON | GESTURE_TWIST_OFF;
    num_buttons = 2;
    strcpy(button_profile, "default");
  }
};

struct RuntimeConfig {
  IniGlobalConfig global;
  IniPreset presets[INI_MAX_PRESETS];
  int num_presets;

  IniAction action_map_on[INI_MAX_SLOTS];
  IniAction action_map_off[INI_MAX_SLOTS];

  bool loaded;

  void SetDefaults() {
    global.SetDefaults();
    num_presets = 1;
    presets[0].SetDefaults();
    loaded = false;

    memset(action_map_on, 0, sizeof(action_map_on));
    memset(action_map_off, 0, sizeof(action_map_off));
  }
};

#endif // PROPS_RUNTIME_CONFIG_H
