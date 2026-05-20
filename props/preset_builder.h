// ProffieOS/props/preset_builder.h
#ifndef PROPS_PRESET_BUILDER_H
#define PROPS_PRESET_BUILDER_H

#include "runtime_config.h"
#include "style_registry.h"
#include "../common/file_reader.h"

#ifndef INI_NUM_BLADES
#define INI_NUM_BLADES 1
#endif

class PresetBuilder {
public:
  static int BuildBladeStyle(const IniPreset* preset,
                             uint8_t blade_idx,
                             char* buf,
                             int buf_size) {
    int len = BuildIniStyleForBlade(preset, blade_idx, buf, buf_size);
    if (len > 0) {
      return len;
    }

    if (blade_idx == 0) {
      const IniStyleEntry* style = FindIniStyle(preset->style_name);
      if (!style) {
        style = &ini_style_registry[0];  // fallback to "standard"
      }
      return style->build(preset, buf, buf_size);
    }

    return len;
  }

  static bool WritePresetsFile(const RuntimeConfig* config, const char* filename) {
    LOCK_SD(true);
    File f = LSFS::OpenForWrite(filename);
    if (!f) {
      LOCK_SD(false);
      return false;
    }

    char style_buf[MAX_STYLE_STRING_LEN];
    f.print("installed=");
    f.print(install_time);
    f.print("\n");

    for (int i = 0; i < config->num_presets; i++) {
      const IniPreset* p = &config->presets[i];

      f.print("new_preset\n");
      f.print("font=");
      f.print(p->font);
      f.print("\n");

      f.print("track=");
      f.print(p->track[0] ? p->track : "");
      f.print("\n");

      uint8_t style_blade_count = p->blade_count;
      if (style_blade_count < 1) style_blade_count = 1;
      if (style_blade_count > INI_NUM_BLADES) style_blade_count = INI_NUM_BLADES;
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

    f.close();
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
