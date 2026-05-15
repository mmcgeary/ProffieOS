// ProffieOS/props/button_profiles.h
#ifndef PROPS_BUTTON_PROFILES_H
#define PROPS_BUTTON_PROFILES_H

#include "runtime_config.h"

enum ButtonSlot {
  SLOT_PWR_CLICK = 0,
  SLOT_PWR_LONG_CLICK = 1,
  SLOT_PWR_HOLD = 2,
  SLOT_PWR_HOLD_LONG = 3,
  SLOT_PWR_DOUBLE_CLICK = 4,
  SLOT_AUX_CLICK = 5,
  SLOT_AUX_LONG_CLICK = 6,
  SLOT_AUX_HOLD = 7,
  SLOT_AUX_HOLD_LONG = 8,
  SLOT_AUX_DOUBLE_CLICK = 9,
  SLOT_PWR_AUX_HOLD = 10,
  SLOT_AUX_PWR_HOLD = 11,
  SLOT_PWR_AUX_CLICK = 12,
  SLOT_AUX2_CLICK = 13,
  SLOT_AUX2_LONG_CLICK = 14,
  SLOT_AUX2_HOLD = 15,
  SLOT_AUX2_HOLD_LONG = 16,
  SLOT_AUX2_DOUBLE_CLICK = 17,
  SLOT_PWR_AUX2_HOLD = 18,
  SLOT_AUX2_PWR_HOLD = 19,
  SLOT_AUX_AUX2_HOLD = 20,
  SLOT_AUX2_AUX_HOLD = 21,
  SLOT_ALL_HOLD = 22,
};

// Default profile — saber ON actions
const IniAction default_profile_on[] = {
  ACTION_OFF,              // 0: PWR click
  ACTION_COLOR_CHANGE,     // 1: PWR long_click
  ACTION_LOCKUP,           // 2: PWR hold
  ACTION_MELT,             // 3: PWR hold_long
  ACTION_FORCE,            // 4: PWR double_click
  ACTION_BLAST,            // 5: AUX click
  ACTION_NEXT_PRESET,      // 6: AUX long_click
  ACTION_LIGHTNING_BLOCK,  // 7: AUX hold
  ACTION_DRAG,             // 8: AUX hold_long
  ACTION_QUOTE,            // 9: AUX double_click
  ACTION_BATTERY_LEVEL,    // 10: PWR+AUX hold
  ACTION_VOLUME_UP,        // 11: AUX+PWR hold
  ACTION_TRACK_PLAYER,     // 12: PWR+AUX click
  ACTION_STAB,             // 13: AUX2 click
  ACTION_PREV_PRESET,      // 14: AUX2 long_click
  ACTION_CLASH,            // 15: AUX2 hold
  ACTION_NONE,             // 16: AUX2 hold_long
  ACTION_NONE,             // 17: AUX2 double_click
  ACTION_NONE,             // 18: PWR+AUX2 hold
  ACTION_NONE,             // 19: AUX2+PWR hold
  ACTION_VOLUME_DOWN,      // 20: AUX+AUX2 hold
  ACTION_NONE,             // 21: AUX2+AUX hold
  ACTION_NONE,             // 22: ALL hold
};

// Default profile — saber OFF actions
const IniAction default_profile_off[] = {
  ACTION_ON,               // 0: PWR click
  ACTION_NEXT_PRESET,      // 1: PWR long_click
  ACTION_NONE,             // 2: PWR hold
  ACTION_NONE,             // 3: PWR hold_long
  ACTION_PREV_PRESET,      // 4: PWR double_click
  ACTION_NEXT_PRESET,      // 5: AUX click
  ACTION_PREV_PRESET,      // 6: AUX long_click
  ACTION_VOLUME_UP,        // 7: AUX hold
  ACTION_VOLUME_DOWN,      // 8: AUX hold_long
  ACTION_TRACK_PLAYER,     // 9: AUX double_click
  ACTION_BATTERY_LEVEL,    // 10: PWR+AUX hold
  ACTION_NONE,             // 11: AUX+PWR hold
  ACTION_NONE,             // 12: PWR+AUX click
  ACTION_BATTERY_LEVEL,    // 13: AUX2 click
  ACTION_TRACK_PLAYER,     // 14: AUX2 long_click
  ACTION_NONE,             // 15: AUX2 hold
  ACTION_NONE,             // 16: AUX2 hold_long
  ACTION_NONE,             // 17: AUX2 double_click
  ACTION_NONE,             // 18: PWR+AUX2 hold
  ACTION_NONE,             // 19: AUX2+PWR hold
  ACTION_NONE,             // 20: AUX+AUX2 hold
  ACTION_NONE,             // 21: AUX2+AUX hold
  ACTION_NONE,             // 22: ALL hold
};

void LoadButtonProfile(const char* profile_name, IniAction* map_on, IniAction* map_off) {
  (void)profile_name;  // reserved for future profiles ("fett263", "sa22c")
  memcpy(map_on, default_profile_on, sizeof(default_profile_on));
  memcpy(map_off, default_profile_off, sizeof(default_profile_off));
}

#endif // PROPS_BUTTON_PROFILES_H
