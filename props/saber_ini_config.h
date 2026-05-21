// ProffieOS/props/saber_ini_config.h
//
// INI-Config Prop for ProffieOS 8.x
// Post-flash configuration via saber_config.ini on SD card.

#ifndef PROPS_SABER_INI_CONFIG_H
#define PROPS_SABER_INI_CONFIG_H

#include <cstring>

inline constexpr const char kReadIniCmd[] = "READ_INI";
inline constexpr const char kWriteIniCmd[] = "WRITE_INI";
inline constexpr const char kReadIniBankCmd[] = "READ_INI_BANK";
inline constexpr const char kWriteIniBankCmd[] = "WRITE_INI_BANK";
inline constexpr const char kBladeInBankArg[] = "blade_in";
inline constexpr const char kBladeOutBankArg[] = "blade_out";
inline constexpr const char kBeginIniMarker[] = "---BEGIN_INI---";
inline constexpr const char kEndIniMarker[] = "---END_INI---";

inline const char* NormalizeIniBankArg(const char* arg) {
  if (!arg || !arg[0]) return nullptr;
  if (!strcmp(arg, kBladeInBankArg)) return kBladeInBankArg;
  if (!strcmp(arg, kBladeOutBankArg)) return kBladeOutBankArg;
  return nullptr;
}

inline bool IsIniStreamControlCommand(const char* cmd) {
  if (!cmd) return false;
  return !strcmp(cmd, kReadIniCmd) ||
         !strcmp(cmd, kWriteIniCmd) ||
         !strcmp(cmd, kReadIniBankCmd) ||
         !strcmp(cmd, kWriteIniBankCmd);
}

#ifndef PROFFIE_TEST

#include "prop_base.h"
#include "ini_parser.h"
#include "color_resolver.h"
#include "runtime_config.h"
#include "ini_loader.h"
#include "style_registry.h"
#include "preset_builder.h"
#include "button_profiles.h"
#include "action_dispatch.h"
#include "blade_bank_utils.h"

#define INI_CONFIG_FILE "saber_config.ini"
#define INI_BLADE_OUT_FILE "blade_out.ini"
#define INI_BUILT_PRESETS_FILE "presets.ini"

#define INI_ALERT_MISSING "ini_missing.wav"
#define INI_ALERT_ERROR "ini_error.wav"
#define INI_ALERT_LOADED "ini_loaded.wav"

class SaberIniConfig : public PropBase {
public:
  SaberIniConfig() : PropBase(), ini_loaded_(false), blade_out_loaded_(false), active_lockup_slot_(-1) {
    blade_in_config_ = nullptr;
    blade_out_config_ = nullptr;
    config_ = nullptr;
  }

  const char* name() override { return "SaberIniConfig"; }

  void Setup() override {
    PropBase::Setup();
    blade_in_config_ = new RuntimeConfig();
    blade_out_config_ = new RuntimeConfig();
    if (blade_in_config_) {
      blade_in_config_->SetDefaults();
      ApplyButtonProfileDefaults(blade_in_config_);
    }
    if (blade_out_config_) {
      blade_out_config_->SetDefaults();
      ApplyButtonProfileDefaults(blade_out_config_);
    }
    config_ = blade_in_config_;
    LoadIniConfig();
    LoadBladeOutConfig();
    SelectActiveConfigForBladeState();
    ApplyGlobalConfig();
  }

  void PlayAlert(const char* filename) {
    if (LSFS::Exists(filename)) beeper.Beep(0.5, 2000);
  }

  void FindBlade(bool announce = false) {
    PropBase::FindBlade(announce);
    if (ini_loaded_) {
      SetPreset(0, false);
      PlayAlert(INI_ALERT_LOADED);
    }
  }

  bool Parse(const char* cmd, const char* arg) override {
    if (streaming_mode_) {
      if (!strcmp(cmd, kEndIniMarker)) {
       streaming_mode_ = false;
       if (stream_file_) {
         stream_file_.close();
       }
       stream_target_file_ = nullptr;
       STDOUT.println("SAVE_OK");
       LOCK_SD(false);
       // Auto-reboot after 500ms to apply changes
       // (Gives serial buffer time to clear SAVE_OK)
       SaberBase::DoEffect(EFFECT_FORCE, 0); // Audible confirmation
       delay(500);
       NVIC_SystemReset();
       return true;
      }

      if (IsIniStreamControlCommand(cmd)) {
       AbortIniStream("ERROR: Command rejected during INI stream");
       return true;
      }

      if (stream_file_) {
       size_t written = 0;
       size_t len = strlen(cmd);
       written += stream_file_.write((const uint8_t*)cmd, len);
       if (arg) {
         written += stream_file_.write((const uint8_t*)" ", 1);
         size_t arg_len = strlen(arg);
         written += stream_file_.write((const uint8_t*)arg, arg_len);
       }
       written += stream_file_.write((const uint8_t*)"\n", 1);
       stream_file_.flush();
      }
      return true;
    }

    if (!strcmp(cmd, kReadIniCmd)) {
      return HandleReadIniBank(kBladeInBankArg);
    }

    if (!strcmp(cmd, kReadIniBankCmd)) {
      return HandleReadIniBank(arg);
    }

    if (!strcmp(cmd, kWriteIniCmd)) {
      return HandleWriteIniBank(kBladeInBankArg);
    }

    if (!strcmp(cmd, kWriteIniBankCmd)) {
      return HandleWriteIniBank(arg);
    }
    
    return PropBase::Parse(cmd, arg);
  }

  bool Event2(enum BUTTON button, EVENT event, uint32_t modifiers) override {
    if (button == BUTTON_NONE && HandleGestureEvent(event, modifiers)) return true;
#ifdef BLADE_DETECT_PIN
    if (button == BUTTON_BLADE_DETECT && HandleBladeDetectEvent(event)) return true;
#endif
    if (event == EVENT_RELEASED) {
      if (active_lockup_slot_ >= 0) {
        EndLockup();
        active_lockup_slot_ = -1;
        return true;
      }
      return false;
    }
    if (!config_) return false;
    const IniAction* action_map = IsOn() ? config_->action_map_on : config_->action_map_off;
    int slot = ResolveButtonSlot(button, event, modifiers, config_->global.num_buttons);
    if (slot < 0) return false;
    IniAction action = action_map[slot];
    if (action == ACTION_NONE) return false;
    if (battle_mode_ && (action == ACTION_LOCKUP || action == ACTION_DRAG || action == ACTION_MELT || action == ACTION_STAB || action == ACTION_LOCKUP_OR_DRAG)) return true;
    if (IsSustainedAction(action) && IsOn()) active_lockup_slot_ = slot;
    ExecuteAction(action, this);
    return true;
  }

  void Loop() override {
    PropBase::Loop();
    if (config_) {
      if (!IsOn() && GestureFlagsNeedOffMotion(config_->global.gesture_flags)) {
        SaberBase::RequestMotion();
      }
      DetectTwist();
    }
    if (!ini_loaded_ && millis() > 3000) LoadIniConfig(); 
  }

  void Off(OffType off_type = OFF_NORMAL, EffectLocation location = EffectLocation()) override {
    battle_mode_ = false; swing_blast_ = false;
    PropBase::Off(off_type, location);
  }

  void ToggleColorChangeMode() { color_change_mode_ = !color_change_mode_; }
  void PlayQuote() { SaberBase::DoEffect(EFFECT_FORCE, 0); }
  void VolumeUp() { if (config_ && dynamic_mixer.get_volume() < VOLUME) { dynamic_mixer.set_volume(std::min<int>(VOLUME, dynamic_mixer.get_volume() + VOLUME / 10)); beeper.Beep(0.5, 2000); } }
  void VolumeDown() { if (config_ && dynamic_mixer.get_volume() > 0) { dynamic_mixer.set_volume(std::max<int>(0, dynamic_mixer.get_volume() - VOLUME / 10)); beeper.Beep(0.5, 1000); } }
  void OnOrVolumeUp() { if (!mode_volume_) On(); else VolumeUp(); }
  void NextPresetOrVolumeDown() { if (!mode_volume_) next_preset(); else VolumeDown(); }
  void PrevPresetIfNotVolumeMenu() { if (!mode_volume_) previous_preset(); }
  void ActivateMuted() { if (SetMute(true)) { unmute_on_deactivation_ = true; On(); } }
  void ToggleVolumeMenu() { mode_volume_ = !mode_volume_; beeper.Beep(0.5, 3000); }
  void ToggleBattleMode() { battle_mode_ = !battle_mode_; beeper.Beep(0.5, battle_mode_ ? 2600 : 1800); }
  void ToggleMultiBlast() { swing_blast_ = !swing_blast_; }
  void ForceOrColorChange() {
#ifndef DISABLE_COLOR_CHANGE
    if (accel_.x < -0.15f) { color_change_mode_ = !color_change_mode_; return; }
#endif
    SaberBase::DoForce();
  }
  void LockupOrDrag() {
    if (SaberBase::Lockup() != SaberBase::LOCKUP_NONE) return;
    if (accel_.x < -0.15f) SaberBase::SetLockup(SaberBase::LOCKUP_DRAG);
    else SaberBase::SetLockup(SaberBase::LOCKUP_NORMAL);
    SaberBase::DoBeginLockup();
  }
  void SayBatteryLevel() { talkie.SayNumber((int)(battery_monitor.battery_percent())); }

private:
  RuntimeConfig* config_;
  RuntimeConfig* blade_in_config_;
  RuntimeConfig* blade_out_config_;
  bool ini_loaded_;
  bool blade_out_loaded_;
  bool color_change_mode_ = false;
  bool mode_volume_ = false;
  bool battle_mode_ = false;
  bool swing_blast_ = false;
  int active_lockup_slot_;
  bool streaming_mode_ = false;
  LSFS::LSFILE stream_file_;
  const char* stream_target_file_ = nullptr;

  void AbortIniStream(const char* error_message) {
    streaming_mode_ = false;
    if (stream_file_) {
      stream_file_.close();
    }
    stream_target_file_ = nullptr;
    if (error_message) {
      STDOUT.println(error_message);
    }
    LOCK_SD(false);
  }

  const char* ResolveIniBankFile(const char* normalized_bank_arg) const {
    return !strcmp(normalized_bank_arg, kBladeOutBankArg) ? INI_BLADE_OUT_FILE : INI_CONFIG_FILE;
  }

  bool HandleReadIniBank(const char* arg) {
    const char* normalized_bank_arg = NormalizeIniBankArg(arg);
    if (!normalized_bank_arg) {
      STDOUT.println("ERROR: Invalid INI bank");
      return true;
    }
    const char* file = ResolveIniBankFile(normalized_bank_arg);
    LOCK_SD(true);
    LSFS::LSFILE f = LSFS::Open(file);
    if (f) {
      STDOUT.println(kBeginIniMarker);
      uint8_t buf[128];
      while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n > 0) STDOUT.write(buf, n);
      }
      STDOUT.println();
      STDOUT.println(kEndIniMarker);
      f.close();
    } else {
      STDOUT.println("ERROR: INI not found");
    }
    LOCK_SD(false);
    return true;
  }

  bool HandleWriteIniBank(const char* arg) {
    const char* normalized_bank_arg = NormalizeIniBankArg(arg);
    if (!normalized_bank_arg) {
      STDOUT.println("ERROR: Invalid INI bank");
      return true;
    }

    LOCK_SD(true);
    if (!LSFS::Begin()) {
      STDOUT.println("ERROR: SD mount failed");
      LOCK_SD(false);
      return true;
    }

    stream_target_file_ = ResolveIniBankFile(normalized_bank_arg);
    stream_file_ = LSFS::OpenForWrite(stream_target_file_);
    if (stream_file_) {
      STDOUT.println("READY_FOR_INI");
      streaming_mode_ = true;
    } else {
      STDOUT.println("ERROR: Open for write failed");
      stream_target_file_ = nullptr;
      LOCK_SD(false);
    }
    return true;
  }

  void LoadIniConfig(bool force = false) {
    static bool tried_loading = false;
    if (tried_loading && !force) return;
    tried_loading = true;
    STDOUT.println("SaberIni: Loading...");
    if (!blade_in_config_) return;
    if (!LSFS::Exists(INI_CONFIG_FILE)) { STDOUT.println("SaberIni: INI missing"); ini_loaded_ = false; return; }
    blade_in_config_->SetDefaults();
    ApplyButtonProfileDefaults(blade_in_config_);
    if (IniLoader::Load(INI_CONFIG_FILE, blade_in_config_)) {
      ApplyGlobalConfig();
      RegeneratePresetBanks();
      ini_loaded_ = true;
      STDOUT.println("SaberIni: LOAD_OK");
    } else {
      STDOUT.println("SaberIni: LOAD_FAIL");
      ini_loaded_ = false;
    }
  }

  void LoadBladeOutConfig() {
    blade_out_loaded_ = false;
    if (!blade_out_config_) return;
    blade_out_config_->SetDefaults();
    ApplyButtonProfileDefaults(blade_out_config_);
    if (!LSFS::Exists(INI_BLADE_OUT_FILE)) return;
    if (IniLoader::Load(INI_BLADE_OUT_FILE, blade_out_config_)) {
      if (blade_in_config_) {
        CopyGlobalAndActions(*blade_in_config_, blade_out_config_);
      }
      blade_out_loaded_ = true;
    }
  }

  void RegeneratePresetBanks() {
    if (!config_) return;
    for (size_t i = 0; i < NELEM(blades); i++) {
      const char* save_dir = blades[i].save_dir ? blades[i].save_dir : "";
      char built_ini[PO_MAXPATH];
      strcpy(built_ini, save_dir);
      if (built_ini[0] && built_ini[strlen(built_ini)-1] != '/') strcat(built_ini, "/");
      strcat(built_ini, INI_BUILT_PRESETS_FILE);
      PresetBuilder::WritePresetsFile(config_, built_ini);
    }
  }

  void SelectActiveConfigForBladeState() {
#ifdef BLADE_DETECT_PIN
    const bool use_blade_out = ShouldUseBladeOutConfig(blade_detected_, blade_out_loaded_);
    config_ = use_blade_out ? blade_out_config_ : blade_in_config_;
#else
    config_ = blade_in_config_;
#endif
  }

  void ApplyGlobalConfig() {
    if (!config_) return;
    uint32_t vol = (uint32_t)config_->global.volume * VOLUME / 100;
    dynamic_mixer.set_volume(vol);
  }

  bool HandleGestureEvent(EVENT event, uint32_t modifiers) {
    if (!config_) return false;
    if (!IsOn()) {
      switch (event) {
        case EVENT_TWIST: if (config_->global.gesture_flags & GESTURE_TWIST_ON) { On(); return true; } break;
        case EVENT_STAB: if (config_->global.gesture_flags & GESTURE_STAB_ON) { On(); return true; } break;
        case EVENT_SWING: if (config_->global.gesture_flags & GESTURE_SWING_ON) { On(); return true; } break;
        case EVENT_THRUST: if (config_->global.gesture_flags & GESTURE_THRUST_ON) { On(); return true; } break;
        default: break;
      }
    } else {
      if (event == EVENT_SWING && swing_blast_) { SaberBase::DoBlast(); return true; }
      switch (event) {
        case EVENT_TWIST: if (config_->global.gesture_flags & GESTURE_TWIST_OFF) { Off(); return true; } break;
        case EVENT_PUSH: if (config_->global.gesture_flags & GESTURE_FORCE_PUSH) { SaberBase::DoForce(); return true; } break;
        case EVENT_STAB: if (config_->global.gesture_flags & GESTURE_MELT) {
            if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) { SaberBase::SetLockup(SaberBase::LOCKUP_MELT); SaberBase::DoBeginLockup(); }
            return true;
          } break;
        default: break;
      }
    }
    return false;
  }

  void EndLockup() { if (SaberBase::Lockup() != SaberBase::LOCKUP_NONE) { SaberBase::DoEndLockup(); SaberBase::SetLockup(SaberBase::LOCKUP_NONE); } }

#ifdef BLADE_DETECT_PIN
  bool HandleBladeDetectEvent(EVENT event) {
    bool blade_inserted = (event == EVENT_LATCH_ON);
    if (event != EVENT_LATCH_ON && event != EVENT_LATCH_OFF) return false;
    blade_detected_ = blade_inserted;
    SelectActiveConfigForBladeState();
    ApplyGlobalConfig();
    FindBladeAgain();
    SaberBase::DoBladeDetect(blade_inserted);
    return true;
  }
#endif
};

#undef PROP_TYPE
#define PROP_TYPE SaberIniConfig

#endif  // !PROFFIE_TEST

#endif // PROPS_SABER_INI_CONFIG_H
