// ProffieOS/props/ini_loader.h
#ifndef PROPS_INI_LOADER_H
#define PROPS_INI_LOADER_H

#include "ini_parser.h"
#include "color_resolver.h"
#include "runtime_config.h"
#include "blade_bank_utils.h"

class IniLoader {
public:
  static bool Load(const char* filename, RuntimeConfig* config) {
    config->SetDefaults();

    IniParser parser;
    if (!parser.Open(filename)) {
      return false;
    }

    int current_preset_idx = -1;
    bool in_global = false;
    bool in_buttons_on = false;
    bool in_buttons_off = false;

    while (true) {
      IniParseResult r = parser.Next();
      if (r == INI_EOF) break;

      if (r == INI_SECTION) {
        in_global = false;
        in_buttons_on = false;
        in_buttons_off = false;
        current_preset_idx = -1;

        if (strcasecmp(parser.Section(), "global") == 0) {
          in_global = true;
        } else if (strncasecmp(parser.Section(), "preset", 6) == 0) {
          int num = atoi(parser.Section() + 6);
          if (num >= 1 && num <= INI_MAX_PRESETS) {
            current_preset_idx = num - 1;
            if (current_preset_idx >= config->num_presets) {
              config->num_presets = current_preset_idx + 1;
            }
            config->presets[current_preset_idx].SetDefaults();
          }
        } else if (strcasecmp(parser.Section(), "buttons_on") == 0) {
          in_buttons_on = true;
        } else if (strcasecmp(parser.Section(), "buttons_off") == 0) {
          in_buttons_off = true;
        }
        continue;
      }

      if (r != INI_KEY_VALUE) continue;

      const char* key = parser.Key();
      const char* val = parser.Value();

      if (in_global) {
        ParseGlobal(key, val, &config->global);
      } else if (in_buttons_on) {
        ParseButtonSlot(key, val, config->action_map_on);
      } else if (in_buttons_off) {
        ParseButtonSlot(key, val, config->action_map_off);
      } else if (current_preset_idx >= 0) {
        ParsePreset(key, val, &config->presets[current_preset_idx]);
      }
    }

    parser.Close();
    FinalizeButtonMappings(config);
    config->loaded = true;
    return true;
  }

private:
  static void FinalizeButtonMappings(RuntimeConfig* config) {
    ApplyButtonProfileDefaults(config);
  }

  static void ParseGlobal(const char* key, const char* val, IniGlobalConfig* g) {
    if (strcasecmp(key, "volume") == 0) {
      g->volume = constrain(atoi(val), 0, 100);
    } else if (strcasecmp(key, "clash_threshold") == 0) {
      g->clash_threshold = constrain(atoi(val), 1, 16);
    } else if (strcasecmp(key, "num_buttons") == 0) {
      g->num_buttons = constrain(atoi(val), 1, 3);
    } else if (strcasecmp(key, "button_profile") == 0) {
      strncpy(g->button_profile, val, INI_MAX_KEY_LEN - 1);
      g->button_profile[INI_MAX_KEY_LEN - 1] = 0;
    } else if (strcasecmp(key, "twist_on") == 0) {
      SetGestureFlag(val, g, GESTURE_TWIST_ON);
    } else if (strcasecmp(key, "twist_off") == 0) {
      SetGestureFlag(val, g, GESTURE_TWIST_OFF);
    } else if (strcasecmp(key, "stab_on") == 0) {
      SetGestureFlag(val, g, GESTURE_STAB_ON);
    } else if (strcasecmp(key, "swing_on") == 0) {
      SetGestureFlag(val, g, GESTURE_SWING_ON);
    } else if (strcasecmp(key, "thrust_on") == 0) {
      SetGestureFlag(val, g, GESTURE_THRUST_ON);
    } else if (strcasecmp(key, "force_push") == 0) {
      SetGestureFlag(val, g, GESTURE_FORCE_PUSH);
    } else if (strcasecmp(key, "melt") == 0) {
      SetGestureFlag(val, g, GESTURE_MELT);
    }
  }

  static void SetGestureFlag(const char* val, IniGlobalConfig* g, uint8_t flag) {
    if (strcasecmp(val, "true") == 0 || strcasecmp(val, "1") == 0 ||
        strcasecmp(val, "yes") == 0 || strcasecmp(val, "on") == 0) {
      g->gesture_flags |= flag;
    } else {
      g->gesture_flags &= ~flag;
    }
  }

  static void ParseOffMode(const char* val, IniPreset* p) {
    if (strcasecmp(val, "pulse") == 0) {
      p->off_mode = OFF_MODE_PULSE;
    } else if (strcasecmp(val, "random") == 0) {
      p->off_mode = OFF_MODE_RANDOM;
    } else if (strcmp(val, "0") == 0) {
      p->off_mode = OFF_MODE_PULSE;
    } else if (strcmp(val, "1") == 0) {
      p->off_mode = OFF_MODE_RANDOM;
    }
  }

  static void ParsePreset(const char* key, const char* val, IniPreset* p) {
    if (strcasecmp(key, "font") == 0) {
      strncpy(p->font, val, INI_MAX_FONT_PATH_LEN - 1);
      p->font[INI_MAX_FONT_PATH_LEN - 1] = 0;
    } else if (strcasecmp(key, "track") == 0) {
      strncpy(p->track, val, INI_MAX_TRACK_PATH_LEN - 1);
      p->track[INI_MAX_TRACK_PATH_LEN - 1] = 0;
    } else if (strcasecmp(key, "style") == 0) {
      strncpy(p->style_name, val, INI_MAX_STYLE_NAME_LEN - 1);
      p->style_name[INI_MAX_STYLE_NAME_LEN - 1] = 0;
    } else if (strcasecmp(key, "name") == 0) {
      strncpy(p->name, val, INI_MAX_KEY_LEN - 1);
      p->name[INI_MAX_KEY_LEN - 1] = 0;
    } else if (strcasecmp(key, "ignition_time") == 0) {
      p->ignition_time = constrain(atoi(val), 50, 2000);
    } else if (strcasecmp(key, "retraction_time") == 0) {
      p->retraction_time = constrain(atoi(val), 50, 2000);
    } else if (strcasecmp(key, "accent_style") == 0) {
      strncpy(p->accent_style, val, INI_MAX_STYLE_NAME_LEN - 1);
      p->accent_style[INI_MAX_STYLE_NAME_LEN - 1] = 0;
    } else if (strcasecmp(key, "accent_speed") == 0) {
      p->accent_speed = constrain(atoi(val), 100, 10000);
    } else if (strcasecmp(key, "flicker_depth") == 0) {
      p->flicker_depth = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "flicker_speed") == 0) {
      p->flicker_speed = constrain(atoi(val), 1, 20000);
    } else if (strcasecmp(key, "stripe_width") == 0) {
      p->stripe_width = constrain(atoi(val), 1, 65535);
    } else if (strcasecmp(key, "stripe_speed") == 0) {
      p->stripe_speed = constrain(atoi(val), 0, 20000);
    } else if (strcasecmp(key, "motion_gain") == 0) {
      p->motion_gain = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "noise_mix") == 0) {
      p->noise_mix = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "base_contrast") == 0 ||
               strcasecmp(key, "core_contrast") == 0) {
      p->base_contrast = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "pulse_rate") == 0) {
      p->pulse_rate = constrain(atoi(val), 1, 20000);
    } else if (strcasecmp(key, "pulse_depth") == 0) {
      p->pulse_depth = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "strobe_freq") == 0) {
      p->strobe_freq = constrain(atoi(val), 1, 200);
    } else if (strcasecmp(key, "strobe_ms") == 0) {
      p->strobe_ms = constrain(atoi(val), 1, 1000);
    } else if (strcasecmp(key, "drift_rate") == 0) {
      p->drift_rate = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "warm_shift") == 0) {
      p->warm_shift = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "jitter_amount") == 0) {
      p->jitter_amount = constrain(atoi(val), 1, 200);
    } else if (strcasecmp(key, "spark_mix") == 0) {
      p->spark_mix = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "heat_rand") == 0) {
      p->heat_rand = constrain(atoi(val), 0, 32768);
    } else if (strcasecmp(key, "fire_cooling") == 0) {
      p->fire_cooling = constrain(atoi(val), 0, 255);
    } else if (strcasecmp(key, "rainbow_speed") == 0) {
      p->rainbow_speed = constrain(atoi(val), 1, 20000);
    } else if (strcasecmp(key, "off_mode") == 0) {
      ParseOffMode(val, p);
    } else if (strcasecmp(key, "off_rate_ms") == 0) {
      p->off_rate_ms = constrain(atoi(val), 10, 60000);
    } else {
      ParseColorField(key, val, p);
    }
  }

  static void ParseColorField(const char* key, const char* val, IniPreset* p) {
    char* target = nullptr;
    if (strcasecmp(key, "base_color") == 0) target = p->base_color;
    else if (strcasecmp(key, "alt_color") == 0) target = p->alt_color;
    else if (strcasecmp(key, "blast_color") == 0) target = p->blast_color;
    else if (strcasecmp(key, "clash_color") == 0) target = p->clash_color;
    else if (strcasecmp(key, "lockup_color") == 0) target = p->lockup_color;
    else if (strcasecmp(key, "drag_color") == 0) target = p->drag_color;
    else if (strcasecmp(key, "lb_color") == 0) target = p->lb_color;
    else if (strcasecmp(key, "stab_color") == 0) target = p->stab_color;
    else if (strcasecmp(key, "swing_color") == 0) target = p->swing_color;
    else if (strcasecmp(key, "emitter_color") == 0) target = p->emitter_color;
    else if (strcasecmp(key, "preon_color") == 0) target = p->preon_color;
    else if (strcasecmp(key, "off_color") == 0) target = p->off_color;

    if (target) {
      ColorToStyleArg(val, target, 20);
    }
  }

  static void ParseButtonSlot(const char* key, const char* val, IniAction* map) {
    if (strncasecmp(key, "slot_", 5) != 0) return;
    int slot = atoi(key + 5);
    if (slot < 0 || slot >= INI_MAX_SLOTS) return;
    map[slot] = LookupAction(val);
  }
};

#endif // PROPS_INI_LOADER_H
