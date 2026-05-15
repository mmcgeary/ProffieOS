// ProffieOS/props/saber_ini_config.h
//
// INI-Config Prop for ProffieOS 8.x
// Post-flash configuration via saber_config.ini on SD card.
// Extends PropBase directly — standalone prop.
//
// Usage in config file:
//   #define PROP_TYPE SaberIniConfig
//   #include "props/saber_ini_config.h"

#ifndef PROPS_SABER_INI_CONFIG_H
#define PROPS_SABER_INI_CONFIG_H

#include "prop_base.h"
#include "ini_parser.h"
#include "color_resolver.h"
#include "runtime_config.h"
#include "ini_loader.h"
#include "style_registry.h"
#include "preset_builder.h"
#include "button_profiles.h"
#include "action_dispatch.h"

#define INI_CONFIG_FILE "saber_config.ini"
#define INI_BLADE_OUT_FILE "blade_out.ini"
#define INI_BUILT_PRESETS_FILE "ini_presets.ini"

#define INI_ALERT_MISSING "ini_missing.wav"
#define INI_ALERT_ERROR "ini_error.wav"
#define INI_ALERT_LOADED "ini_loaded.wav"

class SaberIniConfig : public PropBase {
public:
  SaberIniConfig() : PropBase(), ini_loaded_(false), active_lockup_slot_(-1) {
    config_.SetDefaults();
  }

  const char* name() override { return "SaberIniConfig"; }

  void Setup() override {
    PropBase::Setup();
    LoadIniConfig();
    // Always ensure button defaults are populated for any unset slots.
    // This runs after INI load so INI overrides take precedence.
    ApplyButtonProfile();
  }

  bool Event2(enum BUTTON button, EVENT event, uint32_t modifiers) override {
    // Handle gesture events (BUTTON_NONE with motion events)
    if (button == BUTTON_NONE) {
      return HandleGestureEvent(event, modifiers);
    }

    // Handle button release — end sustained actions
    if (event == EVENT_RELEASED) {
      if (active_lockup_slot_ >= 0) {
        EndLockup();
        active_lockup_slot_ = -1;
        return true;
      }
      return false;
    }

    // Choose action map based on saber state
    const IniAction* action_map = IsOn() ? config_.action_map_on : config_.action_map_off;

    // Resolve event to slot
    int slot = ResolveButtonSlot(button, event, modifiers, config_.global.num_buttons);
    if (slot < 0) return false;

    IniAction action = action_map[slot];
    if (action == ACTION_NONE) return false;

    // Track sustained actions for release handling
    if (IsSustainedAction(action) && IsOn()) {
      active_lockup_slot_ = slot;
    }

    ExecuteAction(action, this);
    return true;
  }

  void Loop() override {
    PropBase::Loop();
  }

  // Public methods called by ExecuteAction
  void ToggleColorChangeMode() {
    color_change_mode_ = !color_change_mode_;
  }

  void PlayQuote() {
    // Trigger quote/force effect from current font
    SaberBase::DoEffect(EFFECT_FORCE, 0);
  }

  void VolumeUp() {
    if (dynamic_mixer.get_volume() < VOLUME) {
      dynamic_mixer.set_volume(std::min<int>(VOLUME, dynamic_mixer.get_volume() + VOLUME / 10));
      beeper.Beep(0.5, 2000);
    }
  }

  void VolumeDown() {
    if (dynamic_mixer.get_volume() > 0) {
      dynamic_mixer.set_volume(std::max<int>(0, dynamic_mixer.get_volume() - VOLUME / 10));
      beeper.Beep(0.5, 1000);
    }
  }

  void SayBatteryLevel() {
    talkie.SayNumber((int)(battery_monitor.battery_percent()));
  }

private:
  RuntimeConfig config_;
  bool ini_loaded_;
  bool color_change_mode_ = false;
  int active_lockup_slot_;

  void LoadIniConfig() {
    if (!LSFS::Exists(INI_CONFIG_FILE)) {
      PlayAlert(INI_ALERT_MISSING);
      ini_loaded_ = false;
      return;
    }

    if (!IniLoader::Load(INI_CONFIG_FILE, &config_)) {
      PlayAlert(INI_ALERT_ERROR);
      ini_loaded_ = false;
      return;
    }

    ApplyGlobalConfig();

    if (!PresetBuilder::WritePresetsFile(&config_, INI_BUILT_PRESETS_FILE)) {
      PlayAlert(INI_ALERT_ERROR);
      ini_loaded_ = false;
      return;
    }

    ini_loaded_ = true;
    SetPreset(0, true);
    PlayAlert(INI_ALERT_LOADED);
  }

  void LoadBladeOutConfig() {
    if (!LSFS::Exists(INI_BLADE_OUT_FILE)) return;

    RuntimeConfig blade_out_config;
    blade_out_config.SetDefaults();
    blade_out_config.global = config_.global;
    memcpy(blade_out_config.action_map_on, config_.action_map_on, sizeof(config_.action_map_on));
    memcpy(blade_out_config.action_map_off, config_.action_map_off, sizeof(config_.action_map_off));

    if (IniLoader::Load(INI_BLADE_OUT_FILE, &blade_out_config)) {
      config_.num_presets = blade_out_config.num_presets;
      memcpy(config_.presets, blade_out_config.presets, sizeof(config_.presets));
      PresetBuilder::WritePresetsFile(&config_, INI_BUILT_PRESETS_FILE);
      SetPreset(0, true);
    }
  }

  void ApplyButtonProfile() {
    IniAction default_on[INI_MAX_SLOTS];
    IniAction default_off[INI_MAX_SLOTS];
    LoadButtonProfile(config_.global.button_profile, default_on, default_off);

    // INI overrides take precedence over profile defaults
    for (int i = 0; i < INI_MAX_SLOTS; i++) {
      if (config_.action_map_on[i] == ACTION_NONE) {
        config_.action_map_on[i] = default_on[i];
      }
      if (config_.action_map_off[i] == ACTION_NONE) {
        config_.action_map_off[i] = default_off[i];
      }
    }
  }

  void ApplyGlobalConfig() {
    uint32_t vol = (uint32_t)config_.global.volume * VOLUME / 100;
    dynamic_mixer.set_volume(vol);
  }

  // Handle gesture events dispatched by prop_base motion detection
  bool HandleGestureEvent(EVENT event, uint32_t modifiers) {
    if (!IsOn()) {
      // OFF gestures — activation
      switch (event) {
        case EVENT_TWIST:
          if (config_.global.gesture_flags & GESTURE_TWIST_ON) { On(); return true; }
          break;
        case EVENT_STAB:
          if (config_.global.gesture_flags & GESTURE_STAB_ON) { On(); return true; }
          break;
        case EVENT_SWING:
          if (config_.global.gesture_flags & GESTURE_SWING_ON) { On(); return true; }
          break;
        case EVENT_THRUST:
          if (config_.global.gesture_flags & GESTURE_THRUST_ON) { On(); return true; }
          break;
        default:
          break;
      }
    } else {
      // ON gestures — deactivation and effects
      switch (event) {
        case EVENT_TWIST:
          if (config_.global.gesture_flags & GESTURE_TWIST_OFF) { Off(); return true; }
          break;
        case EVENT_PUSH:
          if (config_.global.gesture_flags & GESTURE_FORCE_PUSH) {
            SaberBase::DoForce();
            return true;
          }
          break;
        case EVENT_STAB:
          if (config_.global.gesture_flags & GESTURE_MELT) {
            if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) {
              SaberBase::SetLockup(SaberBase::LOCKUP_MELT);
              SaberBase::DoBeginLockup();
            }
            return true;
          }
          break;
        default:
          break;
      }
    }
    return false;
  }

  void EndLockup() {
    if (SaberBase::Lockup() != SaberBase::LOCKUP_NONE) {
      SaberBase::DoEndLockup();
      SaberBase::SetLockup(SaberBase::LOCKUP_NONE);
    }
  }

  void PlayAlert(const char* filename) {
    if (LSFS::Exists(filename)) {
      beeper.Beep(0.5, 2000);
    }
  }
};

#undef PROP_TYPE
#define PROP_TYPE SaberIniConfig

#endif // PROPS_SABER_INI_CONFIG_H
