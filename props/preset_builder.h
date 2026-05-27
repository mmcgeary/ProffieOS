// ProffieOS/props/preset_builder.h
#ifndef PROPS_PRESET_BUILDER_H
#define PROPS_PRESET_BUILDER_H

#include "runtime_config.h"
#include "../common/file_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef INI_NUM_BLADES
#define INI_NUM_BLADES 1
#endif

class PresetBuilder {
public:
  static int BuildBladeStyle(const IniPreset* preset,
                             uint8_t blade_idx,
                             char* buf,
                             int buf_size) {
    const char* style_str = preset->blades[blade_idx].style_name;
    if (!style_str[0]) {
      return snprintf(buf, buf_size, "static 0,0,0");
    } else if (strncmp(style_str, "builtin", 7) == 0) {
      return snprintf(buf, buf_size, "%s", style_str);
    } else {
      int idx = atoi(style_str);
      if (idx >= 0 && style_str[0] >= '0' && style_str[0] <= '9') {
        return snprintf(buf, buf_size, "builtin %d %d", idx, blade_idx + 1);
      } else {
        return snprintf(buf, buf_size, "%s", style_str);
      }
    }
  }

  static bool WritePresetsFile(const RuntimeConfig* config, const char* filename) {
    LOCK_SD(true);
    STDOUT.print("PB: OpenForWrite=");
    STDOUT.println(filename);
    File f = LSFS::OpenForWrite(filename);
    STDOUT.println(f ? "PB: open OK" : "PB: open FAILED");
    if (!f) {
      LOCK_SD(false);
      return false;
    }

    char style_buf[MAX_STYLE_STRING_LEN];
    f.print("installed=");
    f.print(install_time);
    f.print("\n");

    for (int i = 0; i < config->num_presets; i++) {
      STDOUT.print("PB: preset=");
      STDOUT.println(i);
      const IniPreset* p = &config->presets[i];

      f.print("new_preset\n");
      f.print("font=");
      f.print(p->font);
      f.print("\n");

      f.print("track=");
      f.print(p->track[0] ? p->track : "");
      f.print("\n");

      uint8_t style_blade_count = ResolveStyleBladeCount(config, p);
      for (uint8_t blade_idx = 0; blade_idx < style_blade_count; blade_idx++) {
        int len = BuildBladeStyle(p, blade_idx, style_buf, sizeof(style_buf));
        if (len > 0) {
          f.print("style=");
          f.print(style_buf);
          f.print("\n");
        } else {
          f.print("style=static 0,0,0\n");
        }
      }

      f.print("name=");
      f.print(p->name);
      f.print("\n");

      f.print("variation=0\n");
    }
    f.print("end\n");
    STDOUT.println("PB: f.close");
    f.close();
    STDOUT.println("PB: done");
    LOCK_SD(false);
    return true;
  }

  static bool IsValidBuiltStyle(const char* str) {
    if (!str || !str[0]) return false;
    if (str[0] < 'a' || str[0] > 'z') return false;
    const char* p = str;
    while (*p && *p != ' ') {
      if (!((*p >= 'a' && *p <= 'z') || *p == '_')) return false;
      p++;
    }
    while (*p) {
      if (*p != ' ' && *p != ',' && !(*p >= '0' && *p <= '9')) return false;
      p++;
    }
    return true;
  }
};

#endif // PROPS_PRESET_BUILDER_H
