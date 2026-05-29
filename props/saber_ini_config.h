// ProffieOS/props/saber_ini_config.h
//
// INI-Config Prop for ProffieOS 8.x
// Post-flash configuration via saber_config.ini on SD card.

#ifndef PROPS_SABER_INI_CONFIG_H
#define PROPS_SABER_INI_CONFIG_H

#include <cstdio>
#include <cstdint>
#include <cstring>

inline constexpr const char kReadIniCmd[] = "READ_INI";
inline constexpr const char kWriteIniCmd[] = "WRITE_INI";
inline constexpr const char kReadIniBankCmd[] = "READ_INI_BANK";
inline constexpr const char kWriteIniBankCmd[] = "WRITE_INI_BANK";
inline constexpr const char kGetHardwareProfileCmd[] = "GET_HW_PROFILE";
inline constexpr const char kBladeInBankArg[] = "blade_in";
inline constexpr const char kBladeOutBankArg[] = "blade_out";
inline constexpr const char kBeginIniMarker[] = "---BEGIN_INI---";
inline constexpr const char kEndIniMarker[] = "---END_INI---";
inline constexpr uint32_t kIniLoadRetryDisabled = UINT32_MAX;
inline constexpr uint32_t kIniLoadRetryMs = 1000;

inline int NormalizeHardwareCountForProfile(int value) {
  return value > 0 ? value : 1;
}

inline int ResolveHardwareProfileCount(int configured_count, int fallback_count) {
  return configured_count > 0 ? configured_count : NormalizeHardwareCountForProfile(fallback_count);
}

inline int ResolveHardwareBladeCountForProfile() {
#ifdef NUM_BLADES
  return NormalizeHardwareCountForProfile(NUM_BLADES);
#elif defined(INI_NUM_BLADES)
  return NormalizeHardwareCountForProfile(INI_NUM_BLADES);
#else
  return NormalizeHardwareCountForProfile(ResolveRuntimeDefaultBladeCount());
#endif
}

inline int ResolveHardwareButtonCountForProfile() {
#ifdef NUM_BUTTONS
  return NormalizeHardwareCountForProfile(NUM_BUTTONS);
#elif defined(INI_DEFAULT_NUM_BUTTONS)
  return NormalizeHardwareCountForProfile(INI_DEFAULT_NUM_BUTTONS);
#else
  return 1;
#endif
}

inline void BuildHardwareProfileLine(int num_blades,
                                     int num_buttons,
                                     bool has_blade_detect,
                                     bool blade_detected,
                                     char* out,
                                     size_t out_size) {
  if (!out || out_size == 0) return;
  snprintf(out,
           out_size,
           "HW_PROFILE num_blades=%d num_buttons=%d has_blade_detect=%d blade_detect=%d",
           NormalizeHardwareCountForProfile(num_blades),
           NormalizeHardwareCountForProfile(num_buttons),
           has_blade_detect ? 1 : 0,
           blade_detected ? 1 : 0);
}

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

inline bool ShouldAllocateBladeOutConfig() {
#ifdef BLADE_DETECT_PIN
  return true;
#else
  return false;
#endif
}

inline bool ShouldAttemptIniLoad(bool force,
                                 bool ini_loaded,
                                 bool has_runtime_config,
                                 uint32_t now_ms,
                                 uint32_t next_attempt_ms) {
  if (!has_runtime_config) return false;
  if (force) return true;
  if (ini_loaded) return false;
  if (next_attempt_ms == kIniLoadRetryDisabled) return false;
  return static_cast<int32_t>(now_ms - next_attempt_ms) >= 0;
}

inline uint32_t ResolveNextIniLoadAttemptOnMissing(uint32_t now_ms) {
  return now_ms + kIniLoadRetryMs;
}

#ifndef PROFFIE_TEST

uint32_t current_idle_off_time = 600000;
#define IDLE_OFF_TIME current_idle_off_time

uint32_t current_motion_timeout = 900000;
#define MOTION_TIMEOUT current_motion_timeout

uint16_t current_short_click_timeout = 500;
#define BUTTON_SHORT_CLICK_TIMEOUT current_short_click_timeout

uint16_t current_double_click_timeout = 500;
#define BUTTON_DOUBLE_CLICK_TIMEOUT current_double_click_timeout

#include "prop_base.h"
#include "ini_parser.h"
#include "color_resolver.h"
#include "runtime_config.h"
#include "ini_loader.h"

#define INI_CONFIG_FILE "saber_config.ini"
#define INI_BLADE_OUT_FILE "blade_out.ini"
#define INI_BUILT_PRESETS_FILE "presets.ini"

#define INI_ALERT_MISSING "ini_missing.wav"
#define INI_ALERT_ERROR "ini_error.wav"
#define INI_ALERT_LOADED "ini_loaded.wav"

#include "preset_builder.h"
#include "button_profiles.h"
#include "action_dispatch.h"
#include "blade_bank_utils.h"
#include "../styles/ini_style_arg_ids.h"

// Builds a style string of the form:
//   "<base_style_str> <arg1> <arg2> ... <argN>"
// where each positional slot corresponds to the arg ID used by RgbArg/IntArg.
// Slots whose value is unknown/default are emitted as "~" (ArgParser skips ~).
//
// The ArgParser is 1-indexed; we emit args in slot order 1..kExtendedArgCount.
// Color args emit "R,G,B" (8-bit); int args emit decimal integer.
static void BuildArgStyleString(const char* base_style,
                                const IniBladeStyle* blade,
                                char* out, size_t out_size) {
  using namespace ini_style_args;
  // Map from arg-slot index (1-based) -> string to emit.
  // We build it slot by slot; unknown slots get "~".
  const int kMaxSlot = kExtendedArgCount;

  // Start with the base style string.
  size_t pos = strlcpy(out, base_style, out_size);

  char slot_buf[24];

  for (int slot = 1; slot <= kMaxSlot && pos < out_size - 2; slot++) {
    const char* val = nullptr;
    bool is_color = false;

    int color_idx = -1;

    switch (slot) {
      case kBaseColorArg:       color_idx = 0;  is_color = true; break;
      case kAltColorArg:        color_idx = 1;  is_color = true; break;
      case kBlastColorArg:      color_idx = 2;  is_color = true; break;
      case kClashColorArg:      color_idx = 3;  is_color = true; break;
      case kLockupColorArg:     color_idx = 4;  is_color = true; break;
      case kLbColorArg:         color_idx = 5;  is_color = true; break;
      case kDragColorArg:       color_idx = 6;  is_color = true; break;
      case kStabColorArg:       color_idx = 7;  is_color = true; break;
      case kSwingColorArg:      color_idx = 8;  is_color = true; break;
      case kEmitterColorArg:    color_idx = 9;  is_color = true; break;
      case kPreonColorArg:      color_idx = 10; is_color = true; break;
      case kOffColorArg:        color_idx = 11; is_color = true; break;
      case kIgnitionTimeArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->ignition_time);   val = slot_buf; break;
      case kRetractionTimeArg:  snprintf(slot_buf, sizeof(slot_buf), "%u", blade->retraction_time); val = slot_buf; break;
      case kFlickerDepthArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->flicker_depth);   val = slot_buf; break;
      case kFlickerSpeedArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->flicker_speed);   val = slot_buf; break;
      case kStripeWidthArg:     snprintf(slot_buf, sizeof(slot_buf), "%u", blade->stripe_width);    val = slot_buf; break;
      case kStripeSpeedArg:     snprintf(slot_buf, sizeof(slot_buf), "%u", blade->stripe_speed);    val = slot_buf; break;
      case kMotionGainArg:      snprintf(slot_buf, sizeof(slot_buf), "%u", blade->motion_gain);     val = slot_buf; break;
      case kNoiseMixArg:        snprintf(slot_buf, sizeof(slot_buf), "%u", blade->noise_mix);       val = slot_buf; break;
      case kBaseContrastArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->base_contrast);   val = slot_buf; break;
      case kDriftRateArg:       snprintf(slot_buf, sizeof(slot_buf), "%u", blade->drift_rate);      val = slot_buf; break;
      case kWarmShiftArg:       snprintf(slot_buf, sizeof(slot_buf), "%u", blade->warm_shift);      val = slot_buf; break;
      case kJitterAmountArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->jitter_amount);   val = slot_buf; break;
      case kSparkMixArg:        snprintf(slot_buf, sizeof(slot_buf), "%u", blade->spark_mix);       val = slot_buf; break;
      case kHeatRandArg:        snprintf(slot_buf, sizeof(slot_buf), "%u", blade->heat_rand);       val = slot_buf; break;
      case kFireCoolingArg:     snprintf(slot_buf, sizeof(slot_buf), "%u", blade->fire_cooling);    val = slot_buf; break;
      case kRainbowSpeedArg:    snprintf(slot_buf, sizeof(slot_buf), "%u", blade->rainbow_speed);   val = slot_buf; break;
      default:                  break;
    }

    // If a color field, format it.
    if (is_color) {
      if (blade->set_colors_mask & (1 << color_idx)) {
        // Restore 16-bit encoding via * 257 for RgbArg
        snprintf(slot_buf, sizeof(slot_buf), "%u,%u,%u",
                 (unsigned)(blade->colors[color_idx].r * 257),
                 (unsigned)(blade->colors[color_idx].g * 257),
                 (unsigned)(blade->colors[color_idx].b * 257));
        val = slot_buf;
      } else {
        val = nullptr; // failed resolution — fall through to ~
      }
    }

    out[pos++] = ' ';
    if (val && val[0]) {
      size_t len = strlen(val);
      if (pos + len < out_size - 1) {
        memcpy(out + pos, val, len);
        pos += len;
      }
    } else {
      out[pos++] = '~';
    }
  }
  out[pos] = '\0';
}

class SaberIniConfig : public PropBase {
public:
  SaberIniConfig() : PropBase(), ini_loaded_(false), blade_out_loaded_(false), active_lockup_slot_(-1) {
    blade_in_config_ = &blade_in_config_storage_;
    blade_out_config_ = nullptr;
    config_ = nullptr;
  }

  const char* name() override { return "SaberIniConfig"; }

  void Setup() override {
    PropBase::Setup();
    blade_in_config_->SetDefaults();
    ApplyButtonProfileDefaults(blade_in_config_);
    blade_out_config_ = ShouldAllocateBladeOutConfig() ? new RuntimeConfig() : nullptr;
    if (blade_out_config_) {
      blade_out_config_->SetDefaults();
      ApplyButtonProfileDefaults(blade_out_config_);
    }
    config_ = blade_in_config_;
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

  // When INI is loaded, populate current_preset_ directly from INI data
  // without reading or writing presets.ini. This keeps presets.ini as
  // stock ProffieOS expects it (builtin references to config.h presets).
  void SetPreset(int preset_num, bool announce) override {
    if (!ini_loaded_ || !config_ || config_->num_presets == 0) {
      PropBase::SetPreset(preset_num, announce);
      return;
    }

    BladeSet previously_on = BladeOff();
    SaveColorChangeIfNeeded();
    FreeBladeStyles();

    if (config_->num_presets == 0) {
      // Fallback if somehow ini_loaded_ is true but no presets exist
      PropBase::SetPreset(preset_num, init);
      return;
    }

    int idx = ((preset_num % config_->num_presets) + config_->num_presets) % config_->num_presets;
    
    if (config_->active_preset_index != idx) {
      LOCK_SD(true);
      if (IniLoader::LoadPreset(INI_CONFIG_FILE, idx, &config_->active_preset)) {
        config_->active_preset_index = idx;
      } else {
        STDOUT.print("SaberIni: Failed to stream preset ");
        STDOUT.println(idx + 1);
      }
      LOCK_SD(false);
    }
    const IniPreset* p = &config_->active_preset;

    current_preset_.preset_num = idx;
    current_preset_.font = p->font;
    current_preset_.track = (p->track && p->track[0]) ? p->track : "";
    current_preset_.name = (p->name && p->name[0]) ? p->name : "INI Preset";
    current_preset_.variation = 0;

    uint8_t style_blade_count = ResolveStyleBladeCount(config_, p);
    for (int blade = 0; blade < NUM_BLADES; blade++) {
      uint8_t src = (blade < style_blade_count) ? blade : (style_blade_count - 1);
      const IniBladeStyle* blade_style = &p->blades[src];
      const char* style_str = blade_style->style_name;

      // Determine the base "builtin X Y" string.
      char base_buf[32];
      if (!style_str[0]) {
        current_preset_.current_style_[blade] = mkstr("static 0,0,0");
        continue;
      } else if (strncmp(style_str, "builtin", 7) == 0) {
        strlcpy(base_buf, style_str, sizeof(base_buf));
      } else {
        int sidx = atoi(style_str);
        if (sidx >= 0 && style_str[0] >= '0' && style_str[0] <= '9') {
          snprintf(base_buf, sizeof(base_buf), "builtin %d %d", sidx, blade + 1);
        } else {
          strlcpy(base_buf, style_str, sizeof(base_buf));
        }
      }

      // Append all dynamic args so RgbArg/IntArg constructors pick them up.
      // Buffer: base + space + up to kExtendedArgCount args, each up to ~12 chars.
      static char arg_buf[32 + ini_style_args::kExtendedArgCount * 13];
      BuildArgStyleString(base_buf, blade_style, arg_buf, sizeof(arg_buf));
      current_preset_.current_style_[blade] = mkstr(arg_buf);
    }

    AllocateBladeStyles();
    chdir(current_preset_.font.get());
    if (previously_on.on()) FastOn(EffectLocation(0, previously_on));
    if (announce) {
      PVLOG_STATUS << "Current Preset: " << current_preset_.name.get() << "\n";
      SaberBase::DoNewFont();
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

    if (!strcmp(cmd, kGetHardwareProfileCmd)) {
      RuntimeConfig* profile_cfg = blade_in_config_ ? blade_in_config_ : config_;
      const int num_blades = ResolveHardwareBladeCountForProfile();
      const int num_buttons = ResolveHardwareProfileCount(
          profile_cfg ? profile_cfg->global.num_buttons : 0,
          ResolveHardwareButtonCountForProfile());
#ifdef BLADE_DETECT_PIN
      constexpr bool has_blade_detect = true;
      const bool blade_detected = blade_detected_;
#else
      constexpr bool has_blade_detect = false;
      constexpr bool blade_detected = false;
#endif
      char profile_line[160];
      BuildHardwareProfileLine(
          num_blades, num_buttons, has_blade_detect, blade_detected, profile_line, sizeof(profile_line));
      
      if (current_config) {
#define APPEND_BLADE_LENGTH(N) \
        if (current_config->blade##N) { \
          int l = this->GetBladeLength(N); \
          if (l == -1) l = current_config->blade##N->num_leds(); \
          snprintf(profile_line + strlen(profile_line), sizeof(profile_line) - strlen(profile_line), " blade%d_length=%d", N, l); \
        }
        ONCEPERBLADE(APPEND_BLADE_LENGTH);
#undef APPEND_BLADE_LENGTH
      }

      STDOUT.println(profile_line);
      return true;
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
    if (!config_) {
      STDOUT.println("SaberIni: Event2 config_=NULL");
      return false;
    }
    const IniAction* action_map = IsOn() ? config_->action_map_on : config_->action_map_off;
    int slot = ResolveButtonSlot(button, event, modifiers, config_->global.num_buttons);
    if (slot < 0) {
      return false;
    }
    IniAction action = action_map[slot];
    if (action == ACTION_NONE) {
      return false;
    }
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
    if (!ini_loaded_) LoadIniConfig();
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
  RuntimeConfig blade_in_config_storage_;
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
  uint32_t next_ini_load_attempt_ms_ = 0;
  uint8_t ini_retries_ = 0;
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
    const uint32_t now = millis();
    if (!ShouldAttemptIniLoad(force, ini_loaded_, blade_in_config_ != nullptr, now, next_ini_load_attempt_ms_)) return;

#ifdef ENABLE_AUDIO
    // Avoid truncating boot.wav: loading INI calls SetPreset/chdir, which stops active wav players.
    // Defer initial INI load until boot audio has finished.
    if (GetWavPlayerPlaying(&SFX_boot)) {
      next_ini_load_attempt_ms_ = ResolveNextIniLoadAttemptOnMissing(now);
      return;
    }
#endif

    next_ini_load_attempt_ms_ = now + kIniLoadRetryMs;
    STDOUT.println("SaberIni: Loading...");
    if (!blade_in_config_) return;

    STDOUT.println("SaberIni: LOCK_SD");
    LOCK_SD(true);
    STDOUT.println("SaberIni: LSFS::Exists");
    if (!LSFS::Exists(INI_CONFIG_FILE)) {
      LOCK_SD(false);
      if (++ini_retries_ >= 3) {
        STDOUT.println("SaberIni: INI missing (giving up)");
        ini_loaded_ = false; // Must remain false to trigger PropBase::SetPreset fallback
        next_ini_load_attempt_ms_ = kIniLoadRetryDisabled;
      } else {
        STDOUT.println("SaberIni: INI missing (retrying)");
        ini_loaded_ = false;
        next_ini_load_attempt_ms_ = ResolveNextIniLoadAttemptOnMissing(now);
      }
      return;
    }

    STDOUT.println("SaberIni: INI found");
    blade_in_config_->SetDefaults();
    ApplyButtonProfileDefaults(blade_in_config_);
    STDOUT.println("SaberIni: IniLoader::Load");
    const bool loaded = IniLoader::Load(INI_CONFIG_FILE, blade_in_config_);
    STDOUT.println("SaberIni: IniLoader::Load done");
    LOCK_SD(false);

    if (loaded && blade_in_config_->num_presets > 0) {
      STDOUT.print("SaberIni: presets=");
      STDOUT.println(blade_in_config_->num_presets);
      ApplyGlobalConfig();
      ini_loaded_ = true;
      next_ini_load_attempt_ms_ = 0;
      STDOUT.println("SaberIni: LOAD_OK");
      SetPreset(0, false);
#ifdef ENABLE_AUDIO
      SaberBase::DoBoot();
#endif
    } else {
      if (loaded) {
        STDOUT.println("SaberIni: INI has 0 presets (fallback)");
      } else {
        STDOUT.println("SaberIni: LOAD_FAIL");
      }
      ini_loaded_ = false;
      // Do not disable retries here, let the normal retry logic handle it if it wasn't loaded.
      // If it WAS loaded but had 0 presets, we probably don't need to retry, but we'll let it retry just in case it was a partial read.
      if (loaded) {
        next_ini_load_attempt_ms_ = kIniLoadRetryDisabled;
      }
    }
  }

  void LoadBladeOutConfig() {
    blade_out_loaded_ = false;
    if (!blade_out_config_) return;

    LOCK_SD(true);
    blade_out_config_->SetDefaults();
    ApplyButtonProfileDefaults(blade_out_config_);
    if (!LSFS::Exists(INI_BLADE_OUT_FILE)) {
      LOCK_SD(false);
      return;
    }
    if (IniLoader::Load(INI_BLADE_OUT_FILE, blade_out_config_)) {
      if (blade_in_config_) {
        CopyGlobalAndActions(*blade_in_config_, blade_out_config_);
      }
      blade_out_loaded_ = true;
    }
    LOCK_SD(false);
  }

  void RegeneratePresetBanks() {
    if (!config_) return;
    for (size_t i = 0; i < NELEM(blades); i++) {
      const char* save_dir = blades[i].save_dir ? blades[i].save_dir : "";
      char built_ini[PO_MAXPATH];
      strcpy(built_ini, save_dir);
      if (built_ini[0] && built_ini[strlen(built_ini)-1] != '/') strcat(built_ini, "/");
      strcat(built_ini, INI_BUILT_PRESETS_FILE);
      STDOUT.print("SaberIni: WritePresetsFile[");
      STDOUT.print((int)i);
      STDOUT.print("]=");
      STDOUT.println(built_ini);
      bool ok = PresetBuilder::WritePresetsFile(config_, built_ini);
      STDOUT.print("SaberIni: WritePresetsFile[");
      STDOUT.print((int)i);
      STDOUT.print("]=");
      STDOUT.println(ok ? "OK" : "FAIL");
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
    uint32_t vol = std::min<uint32_t>(config_->global.volume, VOLUME);
    dynamic_mixer.set_volume(vol);

#ifdef DYNAMIC_BLADE_DIMMING
    SaberBase::SetDimming((uint32_t)config_->global.blade_dimming * 16384 / 100);
#endif

    current_idle_off_time = config_->global.idle_off_time;
    current_motion_timeout = config_->global.motion_timeout;
    current_short_click_timeout = config_->global.button_short_click_timeout;
    current_double_click_timeout = config_->global.button_double_click_timeout;
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
