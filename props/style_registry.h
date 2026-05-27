// ProffieOS/props/style_registry.h
#ifndef PROPS_STYLE_REGISTRY_H
#define PROPS_STYLE_REGISTRY_H

#include "ini_tuning_arg_table.h"
#include "runtime_config.h"
#include "generated_style_schema.h"
#include "../styles/ini_style_arg_ids.h"
#include <stdarg.h>

#define MAX_STYLE_STRING_LEN 640

#ifndef INI_NUM_BLADES
#define INI_NUM_BLADES 1
#endif

struct IniPreset;

typedef int (*StyleBuildFn)(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size);

struct IniStyleEntry {
  const char* name;
  const char* description;
  StyleBuildFn build;
};

static unsigned int BuildOffModeSelector(const IniPreset* preset) {
  return (preset->off_mode == OFF_MODE_RANDOM) ? 2u : 1u;
}

static unsigned int PulseMillisToRpm(unsigned int pulse_millis) {
  if (pulse_millis == 0u) pulse_millis = 1u;
  const unsigned int pulse_rpm = 60000u / pulse_millis;
  return (pulse_rpm > 0u) ? pulse_rpm : 1u;
}

static unsigned int BuildOffRateArg(const IniPreset* preset) {
  const unsigned int off_rate_ms = (preset->off_rate_ms > 0u) ? preset->off_rate_ms : 1u;
  if (preset->off_mode == OFF_MODE_RANDOM) {
    return off_rate_ms;
  }
  return PulseMillisToRpm(off_rate_ms);
}

static int BuildStyleString(char* buf, int buf_size, const char* format, ...) {
  if (!buf || buf_size <= 0) return -1;

  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buf, buf_size, format, args);
  va_end(args);

  if (written < 0 || written >= buf_size) {
    buf[buf_size - 1] = '\0';
    return -1;
  }
  return written;
}

struct IniTuningArgDef {
  int arg_id;
  uint16_t IniBladeStyle::* member;
};

#define INI_TUNING_ARG_DEF(field, arg_id, key, default_value) \
  { ini_style_args::arg_id, &IniBladeStyle::field },
static constexpr IniTuningArgDef kIniTuningArgDefs[] = {
  INI_TUNING_ARG_TABLE(INI_TUNING_ARG_DEF)
};
#undef INI_TUNING_ARG_DEF

static_assert(kIniTuningArgDefs[0].arg_id == ini_style_args::kFlickerDepthArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[1].arg_id == ini_style_args::kFlickerSpeedArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[2].arg_id == ini_style_args::kStripeWidthArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[3].arg_id == ini_style_args::kStripeSpeedArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[4].arg_id == ini_style_args::kMotionGainArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[5].arg_id == ini_style_args::kNoiseMixArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[6].arg_id == ini_style_args::kBaseContrastArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[7].arg_id == ini_style_args::kDriftRateArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[8].arg_id == ini_style_args::kWarmShiftArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[9].arg_id == ini_style_args::kJitterAmountArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[10].arg_id == ini_style_args::kSparkMixArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[11].arg_id == ini_style_args::kHeatRandArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[12].arg_id == ini_style_args::kFireCoolingArg, "Arg order mismatch.");
static_assert(kIniTuningArgDefs[13].arg_id == ini_style_args::kRainbowSpeedArg, "Arg order mismatch.");
static_assert(
    sizeof(kIniTuningArgDefs) / sizeof(kIniTuningArgDefs[0]) == ini_style_args::kTuningArgCount,
    "Unexpected tuning arg count.");
static_assert(
    ini_style_args::kOffRateMsArg + 1 == ini_style_args::kFirstTuningArg,
    "Expected tuning args to start immediately after off-rate argument.");

static int AppendIniTuningArgs(char* buf, int buf_size, int written, const IniBladeStyle* blade) {
  if (written < 0 || written >= buf_size) return -1;
  const int count = sizeof(kIniTuningArgDefs) / sizeof(kIniTuningArgDefs[0]);
  for (int i = 0; i < count; i++) {
    unsigned int value = static_cast<unsigned int>(blade->*(kIniTuningArgDefs[i].member));
    if (kIniTuningArgDefs[i].arg_id == ini_style_args::kFlickerSpeedArg) {
      value = PulseMillisToRpm(value);
    }
    const int appended = snprintf(buf + written, buf_size - written, " %u", value);
    if (appended < 0 || appended >= buf_size - written) {
      buf[buf_size - 1] = '\0';
      return -1;
    }
    written += appended;
  }
  return written;
}

static int ResolveGeneratedArgIndex(const char* arg_symbol) {
  if (!arg_symbol) return -1;
#define MATCH_SYMBOL(name, arg_id) \
  if (strcmp(arg_symbol, name) == 0) return ini_style_args::arg_id;
  MATCH_SYMBOL("BASE_COLOR_ARG", kBaseColorArg)
  MATCH_SYMBOL("ALT_COLOR_ARG", kAltColorArg)
  MATCH_SYMBOL("STYLE_OPTION_ARG", kStyleOptionArg)
  MATCH_SYMBOL("IGNITION_OPTION_ARG", kIgnitionOptionArg)
  MATCH_SYMBOL("BLAST_COLOR_ARG", kBlastColorArg)
  MATCH_SYMBOL("CLASH_COLOR_ARG", kClashColorArg)
  MATCH_SYMBOL("LOCKUP_COLOR_ARG", kLockupColorArg)
  MATCH_SYMBOL("LB_COLOR_ARG", kLbColorArg)
  MATCH_SYMBOL("DRAG_COLOR_ARG", kDragColorArg)
  MATCH_SYMBOL("STAB_COLOR_ARG", kStabColorArg)
  MATCH_SYMBOL("EMITTER_COLOR_ARG", kEmitterColorArg)
  MATCH_SYMBOL("IGNITION_TIME_ARG", kIgnitionTimeArg)
  MATCH_SYMBOL("RETRACTION_TIME_ARG", kRetractionTimeArg)
  MATCH_SYMBOL("OFF_COLOR_ARG", kOffColorArg)
  MATCH_SYMBOL("OFF_OPTION_ARG", kOffModeArg)
  MATCH_SYMBOL("ALT_COLOR2_ARG", kAltColor2Arg)
  MATCH_SYMBOL("ALT_COLOR3_ARG", kAltColor3Arg)
  MATCH_SYMBOL("STYLE_OPTION2_ARG", kStyleOption2Arg)
  MATCH_SYMBOL("STYLE_OPTION3_ARG", kStyleOption3Arg)
  MATCH_SYMBOL("IGNITION_OPTION2_ARG", kIgnitionOption2Arg)
  MATCH_SYMBOL("RETRACTION_OPTION2_ARG", kRetractionOption2Arg)
  MATCH_SYMBOL("RETRACTION_OPTION_ARG", kRetractionOptionArg)
  MATCH_SYMBOL("SWING_OPTION_ARG", kSwingOptionArg)
  MATCH_SYMBOL("IGNITION_DELAY_ARG", kIgnitionDelayArg)
  MATCH_SYMBOL("RETRACTION_DELAY_ARG", kRetractionDelayArg)
  MATCH_SYMBOL("LOCKUP_POSITION_ARG", kLockupPositionArg)
  MATCH_SYMBOL("DRAG_SIZE_ARG", kDragSizeArg)
  MATCH_SYMBOL("MELT_SIZE_ARG", kMeltSizeArg)
  MATCH_SYMBOL("SWING_COLOR_ARG", kSwingColorArg)
  MATCH_SYMBOL("EMITTER_SIZE_ARG", kEmitterSizeArg)
  MATCH_SYMBOL("PREON_COLOR_ARG", kPreonColorArg)
  MATCH_SYMBOL("PREON_OPTION_ARG", kPreonOptionArg)
  MATCH_SYMBOL("PREON_SIZE_ARG", kPreonSizeArg)
  MATCH_SYMBOL("RETRACTION_COLOR_ARG", kRetractionColorArg)
  MATCH_SYMBOL("RETRACTION_COOL_DOWN_ARG", kRetractionCoolDownArg)
  MATCH_SYMBOL("POSTOFF_COLOR_ARG", kPostOffColorArg)
  MATCH_SYMBOL("IGNITION_COLOR_ARG", kIgnitionColorArg)
  MATCH_SYMBOL("IGNITION_POWER_UP_ARG", kIgnitionPowerUpArg)
#undef MATCH_SYMBOL
  return -1;
}

static const char* ResolveGeneratedKnownStringParam(const IniBladeStyle* blade, const char* key) {
  if (!blade || !key) return nullptr;
  if (strcasecmp(key, "base_color") == 0) return blade->base_color;
  if (strcasecmp(key, "alt_color") == 0) return blade->alt_color;
  if (strcasecmp(key, "blast_color") == 0) return blade->blast_color;
  if (strcasecmp(key, "clash_color") == 0) return blade->clash_color;
  if (strcasecmp(key, "lockup_color") == 0) return blade->lockup_color;
  if (strcasecmp(key, "drag_color") == 0) return blade->drag_color;
  if (strcasecmp(key, "lb_color") == 0) return blade->lb_color;
  if (strcasecmp(key, "stab_color") == 0) return blade->stab_color;
  if (strcasecmp(key, "swing_color") == 0) return blade->swing_color;
  if (strcasecmp(key, "emitter_color") == 0) return blade->emitter_color;
  if (strcasecmp(key, "preon_color") == 0) return blade->preon_color;
  if (strcasecmp(key, "off_color") == 0) return blade->off_color;
  return nullptr;
}

static int BuildGeneratedStyleString(const GeneratedStyleDef* generated,
                                     const IniBladeStyle* blade,
                                     const IniPreset* preset,
                                     char* buf,
                                     int buf_size) {
  if (!generated || !blade || !buf || buf_size <= 0) return -1;

  const int kMaxArg = ini_style_args::kExtendedArgCount;
  const char* arg_tokens[ini_style_args::kExtendedArgCount + 1];
  for (int i = 0; i <= kMaxArg; i++) {
    arg_tokens[i] = "~";
  }
  arg_tokens[0] = generated->parser_name;

  char numeric_tokens[32][16];
  int numeric_index = 0;
  auto set_numeric = [&](int arg_index, unsigned int value) -> bool {
    if (arg_index <= 0 || arg_index > kMaxArg || numeric_index >= 32) return false;
    const int written = snprintf(numeric_tokens[numeric_index], sizeof(numeric_tokens[numeric_index]), "%u", value);
    if (written < 0 || written >= static_cast<int>(sizeof(numeric_tokens[numeric_index]))) return false;
    arg_tokens[arg_index] = numeric_tokens[numeric_index++];
    return true;
  };

  arg_tokens[ini_style_args::kBaseColorArg] = blade->base_color;
  arg_tokens[ini_style_args::kAltColorArg] = blade->alt_color;
  arg_tokens[ini_style_args::kBlastColorArg] = blade->blast_color;
  arg_tokens[ini_style_args::kClashColorArg] = blade->clash_color;
  arg_tokens[ini_style_args::kLockupColorArg] = blade->lockup_color;
  arg_tokens[ini_style_args::kLbColorArg] = blade->lb_color;
  arg_tokens[ini_style_args::kDragColorArg] = blade->drag_color;
  arg_tokens[ini_style_args::kStabColorArg] = blade->stab_color;
  arg_tokens[ini_style_args::kEmitterColorArg] = blade->emitter_color;
  arg_tokens[ini_style_args::kSwingColorArg] = blade->swing_color;
  arg_tokens[ini_style_args::kPreonColorArg] = blade->preon_color;
  arg_tokens[ini_style_args::kOffColorArg] = blade->off_color;

  if (!set_numeric(ini_style_args::kIgnitionTimeArg, blade->ignition_time)) return -1;
  if (!set_numeric(ini_style_args::kRetractionTimeArg, blade->retraction_time)) return -1;
  if (!set_numeric(ini_style_args::kOffModeArg, BuildOffModeSelector(preset))) return -1;
  if (!set_numeric(ini_style_args::kOffRateMsArg, BuildOffRateArg(preset))) return -1;

  if (!set_numeric(ini_style_args::kFlickerDepthArg, blade->flicker_depth)) return -1;
  if (!set_numeric(ini_style_args::kFlickerSpeedArg, PulseMillisToRpm(blade->flicker_speed))) return -1;
  if (!set_numeric(ini_style_args::kStripeWidthArg, blade->stripe_width)) return -1;
  if (!set_numeric(ini_style_args::kStripeSpeedArg, blade->stripe_speed)) return -1;
  if (!set_numeric(ini_style_args::kMotionGainArg, blade->motion_gain)) return -1;
  if (!set_numeric(ini_style_args::kNoiseMixArg, blade->noise_mix)) return -1;
  if (!set_numeric(ini_style_args::kBaseContrastArg, blade->base_contrast)) return -1;
  if (!set_numeric(ini_style_args::kDriftRateArg, blade->drift_rate)) return -1;
  if (!set_numeric(ini_style_args::kWarmShiftArg, blade->warm_shift)) return -1;
  if (!set_numeric(ini_style_args::kJitterAmountArg, blade->jitter_amount)) return -1;
  if (!set_numeric(ini_style_args::kSparkMixArg, blade->spark_mix)) return -1;
  if (!set_numeric(ini_style_args::kHeatRandArg, blade->heat_rand)) return -1;
  if (!set_numeric(ini_style_args::kFireCoolingArg, blade->fire_cooling)) return -1;
  if (!set_numeric(ini_style_args::kRainbowSpeedArg, blade->rainbow_speed)) return -1;

  const int begin = generated->param_offset;
  const int end = generated->param_offset + generated->param_count;
  for (int i = begin; i < end; i++) {
    const GeneratedParamDef& param = kGeneratedParamDefs[i];
    const int arg_index = ResolveGeneratedArgIndex(param.arg_symbol);
    if (arg_index <= 0 || arg_index > kMaxArg) {
      continue;
    }

    const char* override = blade->LookupNamedStyleParam(param.key);
    if (override && override[0]) {
      arg_tokens[arg_index] = override;
      continue;
    }

    const char* known = ResolveGeneratedKnownStringParam(blade, param.key);
    if (known && known[0]) {
      arg_tokens[arg_index] = known;
    }
  }

  int written = snprintf(buf, buf_size, "%s", arg_tokens[0]);
  if (written < 0 || written >= buf_size) {
    buf[buf_size - 1] = '\0';
    return -1;
  }
  for (int i = 1; i <= kMaxArg; i++) {
    const int appended = snprintf(buf + written, buf_size - written, " %s", arg_tokens[i]);
    if (appended < 0 || appended >= buf_size - written) {
      buf[buf_size - 1] = '\0';
      return -1;
    }
    written += appended;
  }
  return written;
}

static int BuildIniStyleWithStringArgs(const char* parser_name,
                                       const IniBladeStyle* blade,
                                       const IniPreset* preset,
                                       char* buf,
                                       int buf_size,
                                       const char* arg1,
                                        const char* arg2,
                                        const char* arg3,
                                        const char* arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %s %s %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    blade->blast_color, blade->clash_color, blade->lockup_color, blade->lb_color,
    blade->drag_color, blade->stab_color, blade->emitter_color,
    blade->ignition_time, blade->retraction_time, blade->off_color,
    BuildOffModeSelector(preset), BuildOffRateArg(preset));
  return AppendIniTuningArgs(buf, buf_size, written, blade);
}

static int BuildIniStyleWithNumericArg34(const char* parser_name,
                                         const IniBladeStyle* blade,
                                         const IniPreset* preset,
                                         char* buf,
                                          int buf_size,
                                          const char* arg1,
                                          const char* arg2,
                                          unsigned int arg3,
                                          unsigned int arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %u %u %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    blade->blast_color, blade->clash_color, blade->lockup_color, blade->lb_color,
    blade->drag_color, blade->stab_color, blade->emitter_color,
    blade->ignition_time, blade->retraction_time, blade->off_color,
    BuildOffModeSelector(preset), BuildOffRateArg(preset));
  return AppendIniTuningArgs(buf, buf_size, written, blade);
}

static int BuildIniStyleWithStringArg3NumericArg4(const char* parser_name,
                                                  const IniBladeStyle* blade,
                                                  const IniPreset* preset,
                                                  char* buf,
                                                   int buf_size,
                                                   const char* arg1,
                                                    const char* arg2,
                                                     const char* arg3,
                                                     unsigned int arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %s %u %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    blade->blast_color, blade->clash_color, blade->lockup_color, blade->lb_color,
    blade->drag_color, blade->stab_color, blade->emitter_color,
    blade->ignition_time, blade->retraction_time, blade->off_color,
    BuildOffModeSelector(preset), BuildOffRateArg(preset));
  return AppendIniTuningArgs(buf, buf_size, written, blade);
}

// --- Main Blade Style Build Functions ---

static int BuildStandard(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_standard", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildHumpFlicker(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_humpflicker", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildUnstable(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_unstable", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->blast_color, blade->clash_color);
}

static int BuildFire(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_fire", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildRainbow(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_rainbow", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildStrobe(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithNumericArg34("ini_strobe", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->strobe_freq, blade->strobe_ms);
}

static int BuildPulse(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithNumericArg34("ini_pulse", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, PulseMillisToRpm(blade->pulse_rate), blade->pulse_depth);
}

static int BuildRotoscope(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_rotoscope", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildGhostly(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_ghostly", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildLightning(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArg3NumericArg4("ini_lightning", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->clash_color, blade->strobe_freq);
}

static int BuildDarksaber(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_darksaber", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildKylo(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_kylo", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->blast_color, blade->clash_color);
}

static int BuildPrequels(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_prequels", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildSequels(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_sequels", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildAncient(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_ancient", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

static int BuildAudioFlicker(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_audioflicker", blade, preset, buf, buf_size,
    blade->base_color, blade->alt_color, blade->swing_color, blade->clash_color);
}

// --- Accent/Crystal Blade Styles ---

static int BuildAccentPulse(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "pulse %s %u", blade->base_color, preset->accent_speed);
}

static int BuildAccentBlink(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "blink %s %u", blade->base_color, preset->accent_speed);
}

static int BuildAccentRandom(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "random %s %u", blade->base_color, preset->accent_speed);
}

static int BuildAccentStatic(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "static %s", blade->base_color);
}

static int BuildAccentColorCycle(const IniBladeStyle* blade, const IniPreset* preset, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "cycle %s %s %s %s %s",
    blade->base_color, blade->base_color, blade->alt_color, blade->blast_color, blade->lockup_color);
}

// --- Registry Tables ---

const IniStyleEntry ini_style_registry[] = {
  {"standard",    "Classic solid blade",           BuildStandard},
  {"humpflicker", "Subtle hump flicker blade",     BuildHumpFlicker},
  {"unstable",    "Flickering unstable blade",     BuildUnstable},
  {"fire",        "Flame animated blade",          BuildFire},
  {"rainbow",     "Cycling rainbow colors",        BuildRainbow},
  {"strobe",      "Strobing flicker blade",        BuildStrobe},
  {"pulse",       "Pulsing glow blade",            BuildPulse},
  {"rotoscope",   "Original trilogy look",         BuildRotoscope},
  {"ghostly",     "Transparent ethereal blade",    BuildGhostly},
  {"lightning",   "Lightning animated blade",      BuildLightning},
  {"darksaber",   "Darksaber high-contrast style", BuildDarksaber},
  {"kylo",        "Crossguard unstable variant",   BuildKylo},
  {"prequels",    "Prequel-era smooth blade",      BuildPrequels},
  {"sequels",     "Sequel-era slight flicker",     BuildSequels},
  {"ancient",     "Ancient Jedi temple style",     BuildAncient},
  {"audioflicker", "Audio-reactive flicker blade",  BuildAudioFlicker},
};

const int INI_STYLE_COUNT = sizeof(ini_style_registry) / sizeof(ini_style_registry[0]);

const IniStyleEntry ini_accent_registry[] = {
  {"pulse",   "Slow pulsing",       BuildAccentPulse},
  {"blink",   "Blinking",           BuildAccentBlink},
  {"random",  "Random flickering",  BuildAccentRandom},
  {"static",  "Always-on static",   BuildAccentStatic},
  {"color_cycle", "Color cycle",    BuildAccentColorCycle},
};

const int INI_ACCENT_STYLE_COUNT = sizeof(ini_accent_registry) / sizeof(ini_accent_registry[0]);

const IniStyleEntry* FindIniStyle(const char* name) {
  for (int i = 0; i < INI_STYLE_COUNT; i++) {
    if (strcasecmp(name, ini_style_registry[i].name) == 0) {
      return &ini_style_registry[i];
    }
  }
  return nullptr;
}

const IniStyleEntry* FindAccentStyle(const char* name) {
  for (int i = 0; i < INI_ACCENT_STYLE_COUNT; i++) {
    if (strcasecmp(name, ini_accent_registry[i].name) == 0) {
      return &ini_accent_registry[i];
    }
  }
  return nullptr;
}

static uint8_t ResolveStyleBladeCount(const RuntimeConfig* config, const IniPreset* preset) {
  uint8_t style_blade_count = 1;
  if (config && config->num_blades > style_blade_count) {
    style_blade_count = config->num_blades;
  }
  if (preset && preset->blade_count > style_blade_count) {
    style_blade_count = preset->blade_count;
  }
  if (style_blade_count > INI_NUM_BLADES) {
    style_blade_count = INI_NUM_BLADES;
  }
  if (style_blade_count > INI_MAX_BLADES) {
    style_blade_count = INI_MAX_BLADES;
  }
  return style_blade_count;
}

static int BuildIniStyleForBlade(const IniPreset* preset,
                                 uint8_t blade_idx,
                                 char* buf,
                                 int buf_size) {
  if (!preset || blade_idx >= INI_MAX_BLADES) {
    return -1;
  }

  uint8_t source_blade = blade_idx;
  if (preset->blade_count == 0 || source_blade >= preset->blade_count) {
    source_blade = 0;
  }

  const IniBladeStyle& blade = preset->blades[source_blade];

  const IniStyleEntry* style = FindIniStyle(blade.style_name);
  if (!style) {
    style = &ini_style_registry[0];
  }

  const GeneratedStyleDef* gen = FindGeneratedStyleDef(blade.style_name);
  if (gen) {
    return BuildGeneratedStyleString(gen, &blade, preset, buf, buf_size);
  }

  return style->build(&blade, preset, buf, buf_size);
}

#endif // PROPS_STYLE_REGISTRY_H
