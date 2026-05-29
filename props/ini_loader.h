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
          if (num >= 1) {
            current_preset_idx = num - 1;
            if (current_preset_idx >= config->num_presets) {
              config->num_presets = current_preset_idx + 1;
            }
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
      }
      // Intentionally skipping presets in the main Load() to save RAM.
      // Presets are loaded on-demand via LoadPreset().
    }

    parser.Close();
    FinalizeButtonMappings(config);
    config->loaded = true;
    return true;
  }

  static bool LoadPreset(const char* filename, int target_idx, IniPreset* out_preset) {
    IniParser parser;
    if (!parser.Open(filename)) {
      return false;
    }

    out_preset->SetDefaults();
    
    char target_section[32];
    snprintf(target_section, sizeof(target_section), "preset%d", target_idx + 1);

    bool in_target_preset = false;
    bool found_preset = false;

    while (true) {
      IniParseResult r = parser.Next();
      if (r == INI_EOF) break;

      if (r == INI_SECTION) {
        if (strcasecmp(parser.Section(), target_section) == 0) {
          in_target_preset = true;
          found_preset = true;
        } else {
          // If we were in the target preset and hit a new section, we are done.
          if (in_target_preset) break;
        }
        continue;
      }

      if (r != INI_KEY_VALUE) continue;

      if (in_target_preset) {
        ParsePreset(parser.Key(), parser.Value(), out_preset);
      }
    }

    parser.Close();
    return found_preset;
  }

private:
  static void FinalizeButtonMappings(RuntimeConfig* config) {
    ApplyButtonProfileDefaults(config);
  }

  static void ParseGlobal(const char* key, const char* val, IniGlobalConfig* g) {
    if (strcasecmp(key, "volume") == 0) {
      g->volume = constrain(atoi(val), 0, 3000);
    } else if (strcasecmp(key, "max_volume") == 0) {
      g->max_volume = constrain(atoi(val), 0, 3000);
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
    } else if (strcasecmp(key, "blade_dimming") == 0) {
      g->blade_dimming = constrain(atoi(val), 0, 100);
    } else if (strcasecmp(key, "idle_off_time") == 0) {
      g->idle_off_time = constrain(atoi(val), 0, 86400000);
    } else if (strcasecmp(key, "motion_timeout") == 0) {
      g->motion_timeout = constrain(atoi(val), 0, 86400000);
    } else if (strcasecmp(key, "button_short_click_timeout") == 0) {
      g->button_short_click_timeout = constrain(atoi(val), 50, 2000);
    } else if (strcasecmp(key, "button_double_click_timeout") == 0) {
      g->button_double_click_timeout = constrain(atoi(val), 50, 2000);
    } else if (strncasecmp(key, "blade", 5) == 0 && strstr(key, "_length") != nullptr) {
      int blade_num = atoi(key + 5);
      if (blade_num >= 1 && blade_num <= 10) {
        g->blade_length[blade_num - 1] = atoi(val);
      }
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

  static bool ParseBladeKey(const char* key, int* blade_idx, const char** blade_key) {
    if (strncasecmp(key, "blade", 5) != 0) return false;
    const char* p = key + 5;
    if (*p < '0' || *p > '9') return false;

    int blade_num = 0;
    while (*p >= '0' && *p <= '9') {
      blade_num = blade_num * 10 + (*p - '0');
      ++p;
    }

    if (blade_num < 1 || *p != '_' || *(p + 1) == 0) return false;
    *blade_idx = blade_num - 1;
    *blade_key = p + 1;
    return true;
  }

  static bool ParseColorField(const char* key, const char* val, IniBladeStyle* blade) {
    int color_idx = -1;
    if (strcasecmp(key, "base_color") == 0) color_idx = 0;
    else if (strcasecmp(key, "alt_color") == 0) color_idx = 1;
    else if (strcasecmp(key, "blast_color") == 0) color_idx = 2;
    else if (strcasecmp(key, "clash_color") == 0) color_idx = 3;
    else if (strcasecmp(key, "lockup_color") == 0) color_idx = 4;
    else if (strcasecmp(key, "drag_color") == 0) color_idx = 5;
    else if (strcasecmp(key, "lb_color") == 0) color_idx = 6;
    else if (strcasecmp(key, "stab_color") == 0) color_idx = 7;
    else if (strcasecmp(key, "swing_color") == 0) color_idx = 8;
    else if (strcasecmp(key, "emitter_color") == 0) color_idx = 9;
    else if (strcasecmp(key, "preon_color") == 0) color_idx = 10;
    else if (strcasecmp(key, "off_color") == 0) color_idx = 11;

    if (color_idx < 0) return false;

    uint16_t r, g, b;
    if (!ResolveColor(val, &r, &g, &b)) {
      r = g = b = 65535;
    }
    blade->colors[color_idx].r = r >> 8;
    blade->colors[color_idx].g = g >> 8;
    blade->colors[color_idx].b = b >> 8;
    blade->set_colors_mask |= (1 << color_idx);
    return true;
  }

  static bool ParseBladeField(const char* key, const char* val, IniBladeStyle* blade) {
    if (strncasecmp(key, "param.", 6) == 0) {
      const char* param_name = key + 6;
      if (!param_name[0]) return false;
      return blade->SetNamedStyleParam(param_name, val);
    }

    if (strcasecmp(key, "style") == 0) {
      strncpy(blade->style_name, val, INI_MAX_STYLE_NAME_LEN - 1);
      blade->style_name[INI_MAX_STYLE_NAME_LEN - 1] = 0;
      return true;
    } else if (strcasecmp(key, "ignition_time") == 0) {
      blade->ignition_time = constrain(atoi(val), 50, 2000);
      return true;
    } else if (strcasecmp(key, "retraction_time") == 0) {
      blade->retraction_time = constrain(atoi(val), 50, 2000);
      return true;
    } else if (strcasecmp(key, "flicker_depth") == 0) {
      blade->flicker_depth = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "flicker_speed") == 0) {
      blade->flicker_speed = constrain(atoi(val), 1, 20000);
      return true;
    } else if (strcasecmp(key, "stripe_width") == 0) {
      blade->stripe_width = constrain(atoi(val), 1, 65535);
      return true;
    } else if (strcasecmp(key, "stripe_speed") == 0) {
      blade->stripe_speed = constrain(atoi(val), 0, 20000);
      return true;
    } else if (strcasecmp(key, "motion_gain") == 0) {
      blade->motion_gain = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "noise_mix") == 0) {
      blade->noise_mix = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "base_contrast") == 0 ||
               strcasecmp(key, "core_contrast") == 0) {
      blade->base_contrast = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "pulse_rate") == 0) {
      blade->pulse_rate = constrain(atoi(val), 1, 20000);
      return true;
    } else if (strcasecmp(key, "pulse_depth") == 0) {
      blade->pulse_depth = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "strobe_freq") == 0) {
      blade->strobe_freq = constrain(atoi(val), 1, 200);
      return true;
    } else if (strcasecmp(key, "strobe_ms") == 0) {
      blade->strobe_ms = constrain(atoi(val), 1, 1000);
      return true;
    } else if (strcasecmp(key, "drift_rate") == 0) {
      blade->drift_rate = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "warm_shift") == 0) {
      blade->warm_shift = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "jitter_amount") == 0) {
      blade->jitter_amount = constrain(atoi(val), 1, 200);
      return true;
    } else if (strcasecmp(key, "spark_mix") == 0) {
      blade->spark_mix = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "heat_rand") == 0) {
      blade->heat_rand = constrain(atoi(val), 0, 32768);
      return true;
    } else if (strcasecmp(key, "fire_cooling") == 0) {
      blade->fire_cooling = constrain(atoi(val), 0, 255);
      return true;
    } else if (strcasecmp(key, "rainbow_speed") == 0) {
      blade->rainbow_speed = constrain(atoi(val), 1, 20000);
      return true;
    }
    return ParseColorField(key, val, blade);
  }

  static void ParsePreset(const char* key, const char* val, IniPreset* p) {
    int blade_idx = -1;
    const char* blade_key = nullptr;
    if (ParseBladeKey(key, &blade_idx, &blade_key)) {
      if (blade_idx >= 0 && blade_idx < INI_MAX_BLADES &&
          ParseBladeField(blade_key, val, &p->blades[blade_idx])) {
        if (blade_idx + 1 > p->blade_count) {
          p->blade_count = blade_idx + 1;
        }
      }
      return;
    }

    if (strcasecmp(key, "font") == 0) {
      strncpy(p->font, val, INI_MAX_FONT_PATH_LEN - 1);
      p->font[INI_MAX_FONT_PATH_LEN - 1] = 0;
    } else if (strcasecmp(key, "track") == 0) {
      strncpy(p->track, val, INI_MAX_TRACK_PATH_LEN - 1);
      p->track[INI_MAX_TRACK_PATH_LEN - 1] = 0;
    } else if (strcasecmp(key, "name") == 0) {
      strncpy(p->name, val, INI_MAX_KEY_LEN - 1);
      p->name[INI_MAX_KEY_LEN - 1] = 0;
    } else if (strcasecmp(key, "accent_style") == 0) {
      strncpy(p->accent_style, val, INI_MAX_STYLE_NAME_LEN - 1);
      p->accent_style[INI_MAX_STYLE_NAME_LEN - 1] = 0;
    } else if (strcasecmp(key, "accent_speed") == 0) {
      p->accent_speed = constrain(atoi(val), 100, 10000);
    } else if (strcasecmp(key, "off_mode") == 0) {
      ParseOffMode(val, p);
    } else if (strcasecmp(key, "off_rate_ms") == 0) {
      p->off_rate_ms = constrain(atoi(val), 10, 60000);
    } else if (ParseBladeField(key, val, &p->blades[0])) {
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
