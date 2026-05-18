// ProffieOS/props/button_profiles.h
#ifndef PROPS_BUTTON_PROFILES_H
#define PROPS_BUTTON_PROFILES_H

#include "runtime_config.h"
#include <string.h>

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
  SLOT_PWR_HOLD_MEDIUM = 23,
  SLOT_PWR_DOUBLE_HOLD = 24,
  SLOT_PWR_TRIPLE_CLICK = 25,
  SLOT_PWR_TRIPLE_HOLD = 26,
  SLOT_AUX_DOUBLE_HOLD = 27,
  SLOT_AUX_TRIPLE_CLICK = 28,
  SLOT_AUX2_DOUBLE_HOLD = 29,
  SLOT_PWR_MOD_CLASH = 30,
  SLOT_PWR_MOD_STAB = 31,
  SLOT_PWR_MOD_SWING = 32,
  SLOT_PWR_MOD_TWIST = 33,
};

inline void ClearProfile(IniAction* map_on, IniAction* map_off) {
  for (int i = 0; i < INI_MAX_SLOTS; i++) {
    map_on[i] = ACTION_NONE;
    map_off[i] = ACTION_NONE;
  }
}

inline void LoadSa22cOneButtonProfile(IniAction* map_on, IniAction* map_off) {
  map_off[SLOT_PWR_CLICK] = ACTION_ON_OR_VOLUME_UP;
  map_off[SLOT_PWR_LONG_CLICK] = ACTION_NEXT_PRESET_OR_VOLUME_DOWN;
  map_off[SLOT_PWR_HOLD_LONG] = ACTION_PREV_PRESET_IF_NOT_VOLUME_MENU;
  map_off[SLOT_PWR_DOUBLE_CLICK] = ACTION_TRACK_PLAYER;
  map_off[SLOT_PWR_DOUBLE_HOLD] = ACTION_ACTIVATE_MUTED;
  map_off[SLOT_PWR_TRIPLE_CLICK] = ACTION_BATTERY_LEVEL;
  map_off[SLOT_PWR_MOD_CLASH] = ACTION_TOGGLE_VOLUME_MENU;

  map_on[SLOT_PWR_CLICK] = ACTION_BLAST;
  map_on[SLOT_PWR_HOLD_LONG] = ACTION_OFF;
  map_on[SLOT_PWR_DOUBLE_CLICK] = ACTION_BLAST;
  map_on[SLOT_PWR_DOUBLE_HOLD] = ACTION_LIGHTNING_BLOCK;
  map_on[SLOT_PWR_TRIPLE_CLICK] = ACTION_BLAST;
  map_on[SLOT_PWR_TRIPLE_HOLD] = ACTION_TOGGLE_BATTLE_MODE;
  map_on[SLOT_PWR_MOD_CLASH] = ACTION_LOCKUP_OR_DRAG;
  map_on[SLOT_PWR_MOD_STAB] = ACTION_MELT;
  map_on[SLOT_PWR_MOD_SWING] = ACTION_TOGGLE_MULTI_BLAST;
  map_on[SLOT_PWR_MOD_TWIST] = ACTION_FORCE_OR_COLOR_CHANGE;
}

inline void LoadSa22cTwoButtonProfile(IniAction* map_on, IniAction* map_off) {
  map_off[SLOT_PWR_CLICK] = ACTION_ON_OR_VOLUME_UP;
  map_off[SLOT_PWR_LONG_CLICK] = ACTION_TRACK_PLAYER;
  map_off[SLOT_PWR_HOLD_LONG] = ACTION_PREV_PRESET_IF_NOT_VOLUME_MENU;
  map_off[SLOT_PWR_DOUBLE_HOLD] = ACTION_ACTIVATE_MUTED;
  map_off[SLOT_AUX_CLICK] = ACTION_NEXT_PRESET_OR_VOLUME_DOWN;
  map_off[SLOT_AUX_LONG_CLICK] = ACTION_TOGGLE_VOLUME_MENU;
  map_off[SLOT_AUX_HOLD_LONG] = ACTION_BATTERY_LEVEL;

  map_on[SLOT_PWR_HOLD_MEDIUM] = ACTION_OFF;
  map_on[SLOT_PWR_DOUBLE_CLICK] = ACTION_FORCE;
  map_on[SLOT_PWR_DOUBLE_HOLD] = ACTION_LIGHTNING_BLOCK;
  map_on[SLOT_PWR_TRIPLE_HOLD] = ACTION_TOGGLE_BATTLE_MODE;
  map_on[SLOT_AUX_CLICK] = ACTION_BLAST;
  map_on[SLOT_AUX_HOLD] = ACTION_LOCKUP_OR_DRAG;
  map_on[SLOT_AUX_DOUBLE_CLICK] = ACTION_BLAST;
  map_on[SLOT_AUX_DOUBLE_HOLD] = ACTION_TOGGLE_MULTI_BLAST;
  map_on[SLOT_AUX_TRIPLE_CLICK] = ACTION_BLAST;
  map_on[SLOT_PWR_AUX_CLICK] = ACTION_COLOR_CHANGE;
  map_on[SLOT_PWR_MOD_STAB] = ACTION_MELT;
}

inline void LoadSa22cThreeButtonProfile(IniAction* map_on, IniAction* map_off) {
  LoadSa22cTwoButtonProfile(map_on, map_off);
  map_off[SLOT_AUX2_CLICK] = ACTION_PREV_PRESET;
  map_on[SLOT_AUX2_HOLD] = ACTION_LIGHTNING_BLOCK;
  map_on[SLOT_AUX2_DOUBLE_HOLD] = ACTION_TOGGLE_BATTLE_MODE;
  map_on[SLOT_PWR_TRIPLE_HOLD] = ACTION_NONE;
}

void LoadButtonProfile(const char* profile_name,
                       uint8_t num_buttons,
                       IniAction* map_on,
                       IniAction* map_off) {
  const bool sa22c =
      !profile_name ||
      strcasecmp(profile_name, "default") == 0 ||
      strcasecmp(profile_name, "sa22c") == 0;
  (void)sa22c;

  ClearProfile(map_on, map_off);
  if (num_buttons >= 3) {
    LoadSa22cThreeButtonProfile(map_on, map_off);
  } else if (num_buttons == 2) {
    LoadSa22cTwoButtonProfile(map_on, map_off);
  } else {
    LoadSa22cOneButtonProfile(map_on, map_off);
  }
}

#endif // PROPS_BUTTON_PROFILES_H
