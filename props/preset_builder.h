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
  static int BuildMainStyle(const IniPreset* preset, char* buf, int buf_size) {
    const IniStyleEntry* style = FindIniStyle(preset->style_name);
    if (!style) {
      style = &ini_style_registry[0];  // fallback to "standard"
    }
    return style->build(preset, buf, buf_size);
  }

  static int BuildAccentStyle(const IniPreset* preset, char* buf, int buf_size) {
    if (preset->accent_style[0] == 0) return 0;

    const IniStyleEntry* style = FindAccentStyle(preset->accent_style);
    if (!style) {
      style = &ini_accent_registry[3];  // fallback to "static"
    }
    return style->build(preset, buf, buf_size);
  }

  static bool WritePresetsFile(const RuntimeConfig* config, const char* filename) {
    LOCK_SD(true);
    File f = LSFS::Open(filename, FILE_WRITE | O_TRUNC);
    if (!f) {
      LOCK_SD(false);
      return false;
    }

    char style_buf[MAX_STYLE_STRING_LEN];

    for (int i = 0; i < config->num_presets; i++) {
      const IniPreset* p = &config->presets[i];

      f.print("new_preset\n");
      f.print("font=");
      f.print(p->font);
      f.print("\n");

      f.print("track=");
      f.print(p->track[0] ? p->track : "");
      f.print("\n");

      // Main blade style
      int len = BuildMainStyle(p, style_buf, sizeof(style_buf));
      if (len > 0) {
        f.print("style=");
        f.print(style_buf);
        f.print("\n");
      }

      // Additional blades (accent/crystal) if configured
#if INI_NUM_BLADES > 1
      len = BuildAccentStyle(p, style_buf, sizeof(style_buf));
      if (len > 0) {
        f.print("style=");
        f.print(style_buf);
        f.print("\n");
      } else {
        f.print("style=static 0,0,0\n");
      }
#endif

      f.print("name=");
      f.print(p->name);
      f.print("\n");

      f.print("variation=0\n");
      f.print("end\n");
    }

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
