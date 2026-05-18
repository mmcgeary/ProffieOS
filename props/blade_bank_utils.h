#ifndef PROPS_BLADE_BANK_UTILS_H
#define PROPS_BLADE_BANK_UTILS_H

#include "button_profiles.h"
#include "runtime_config.h"
#include "../common/lsfs.h"
#include <string.h>

inline bool ShouldUseBladeOutConfig(bool blade_detected, bool blade_out_exists) {
  return !blade_detected && blade_out_exists;
}

inline void CopyGlobalAndActions(const RuntimeConfig& src, RuntimeConfig* dst) {
  dst->global = src.global;
  memcpy(dst->action_map_on, src.action_map_on, sizeof(src.action_map_on));
  memcpy(dst->action_map_off, src.action_map_off, sizeof(src.action_map_off));
}

inline void ApplyButtonProfileDefaults(RuntimeConfig* runtime) {
  IniAction default_on[INI_MAX_SLOTS];
  IniAction default_off[INI_MAX_SLOTS];
  LoadButtonProfile(runtime->global.button_profile,
                    runtime->global.num_buttons,
                    default_on,
                    default_off);

  for (int i = 0; i < INI_MAX_SLOTS; i++) {
    if (runtime->action_map_on[i] == ACTION_NONE) {
      runtime->action_map_on[i] = default_on[i];
    }
    if (runtime->action_map_off[i] == ACTION_NONE) {
      runtime->action_map_off[i] = default_off[i];
    }
  }
}

inline void BuildSaveDirPath(const char* save_dir,
                             const char* filename,
                             char* out,
                             size_t out_size) {
  if (!out_size) return;
  PathHelper full_path(save_dir ? save_dir : "", filename);
  strncpy(out, full_path, out_size - 1);
  out[out_size - 1] = 0;
}

#endif  // PROPS_BLADE_BANK_UTILS_H
