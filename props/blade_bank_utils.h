#ifndef PROPS_BLADE_BANK_UTILS_H
#define PROPS_BLADE_BANK_UTILS_H

#include "runtime_config.h"
#include <string.h>

inline bool ShouldUseBladeOutConfig(bool blade_detected, bool blade_out_exists) {
  return !blade_detected && blade_out_exists;
}

inline void CopyGlobalAndActions(const RuntimeConfig& src, RuntimeConfig* dst) {
  dst->global = src.global;
  memcpy(dst->action_map_on, src.action_map_on, sizeof(src.action_map_on));
  memcpy(dst->action_map_off, src.action_map_off, sizeof(src.action_map_off));
}

#endif  // PROPS_BLADE_BANK_UTILS_H
