// ProffieOS/props/style_registry.h
#ifndef PROPS_STYLE_REGISTRY_H
#define PROPS_STYLE_REGISTRY_H

#include "ini_tuning_arg_table.h"
#include "runtime_config.h"
#include <stdarg.h>

#define MAX_STYLE_STRING_LEN 640

struct IniPreset;

typedef int (*StyleBuildFn)(const IniPreset* preset, char* buf, int buf_size);

struct IniStyleEntry {
  const char* name;
  const char* description;
  StyleBuildFn build;
};

static unsigned int BuildOffModeSelector(const IniPreset* p) {
  return (p->off_mode == OFF_MODE_RANDOM) ? 2u : 1u;
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
  uint16_t IniPreset::* member;
};

#define INI_TUNING_ARG_DEF(field, arg_id, key, default_value) \
  { ini_style_args::arg_id, &IniPreset::field },
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

static int AppendIniTuningArgs(char* buf, int buf_size, int written, const IniPreset* p) {
  if (written < 0 || written >= buf_size) return -1;
  const int count = sizeof(kIniTuningArgDefs) / sizeof(kIniTuningArgDefs[0]);
  for (int i = 0; i < count; i++) {
    const unsigned int value = static_cast<unsigned int>(p->*(kIniTuningArgDefs[i].member));
    const int appended = snprintf(buf + written, buf_size - written, " %u", value);
    if (appended < 0 || appended >= buf_size - written) {
      buf[buf_size - 1] = '\0';
      return -1;
    }
    written += appended;
  }
  return written;
}

static int BuildIniStyleWithStringArgs(const char* parser_name,
                                       const IniPreset* p,
                                       char* buf,
                                       int buf_size,
                                       const char* arg1,
                                        const char* arg2,
                                        const char* arg3,
                                        const char* arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %s %s %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    p->blast_color, p->clash_color, p->lockup_color, p->lb_color,
    p->drag_color, p->stab_color, p->emitter_color,
    p->ignition_time, p->retraction_time, p->off_color,
    BuildOffModeSelector(p), p->off_rate_ms);
  return AppendIniTuningArgs(buf, buf_size, written, p);
}

static int BuildIniStyleWithNumericArg34(const char* parser_name,
                                         const IniPreset* p,
                                         char* buf,
                                          int buf_size,
                                          const char* arg1,
                                          const char* arg2,
                                          unsigned int arg3,
                                          unsigned int arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %u %u %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    p->blast_color, p->clash_color, p->lockup_color, p->lb_color,
    p->drag_color, p->stab_color, p->emitter_color,
    p->ignition_time, p->retraction_time, p->off_color,
    BuildOffModeSelector(p), p->off_rate_ms);
  return AppendIniTuningArgs(buf, buf_size, written, p);
}

static int BuildIniStyleWithStringArg3NumericArg4(const char* parser_name,
                                                  const IniPreset* p,
                                                  char* buf,
                                                   int buf_size,
                                                   const char* arg1,
                                                    const char* arg2,
                                                     const char* arg3,
                                                     unsigned int arg4) {
  const int written = BuildStyleString(buf, buf_size, "%s %s %s %s %u %s %s %s %s %s %s %s %u %u %s %u %u",
    parser_name,
    arg1, arg2, arg3, arg4,
    p->blast_color, p->clash_color, p->lockup_color, p->lb_color,
    p->drag_color, p->stab_color, p->emitter_color,
    p->ignition_time, p->retraction_time, p->off_color,
    BuildOffModeSelector(p), p->off_rate_ms);
  return AppendIniTuningArgs(buf, buf_size, written, p);
}

// --- Main Blade Style Build Functions ---

static int BuildStandard(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_standard", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildHumpFlicker(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_humpflicker", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildUnstable(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_unstable", p, buf, buf_size,
    p->base_color, p->alt_color, p->blast_color, p->clash_color);
}

static int BuildFire(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_fire", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildRainbow(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_rainbow", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildStrobe(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithNumericArg34("ini_strobe", p, buf, buf_size,
    p->base_color, p->alt_color, p->strobe_freq, p->strobe_ms);
}

static int BuildPulse(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithNumericArg34("ini_pulse", p, buf, buf_size,
    p->base_color, p->alt_color, p->pulse_rate, p->pulse_depth);
}

static int BuildRotoscope(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_rotoscope", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildGhostly(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_ghostly", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildLightning(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArg3NumericArg4("ini_lightning", p, buf, buf_size,
    p->base_color, p->alt_color, p->clash_color, p->strobe_freq);
}

static int BuildDarksaber(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_darksaber", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildKylo(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_kylo", p, buf, buf_size,
    p->base_color, p->alt_color, p->blast_color, p->clash_color);
}

static int BuildPrequels(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_prequels", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildSequels(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_sequels", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

static int BuildAncient(const IniPreset* p, char* buf, int buf_size) {
  return BuildIniStyleWithStringArgs("ini_ancient", p, buf, buf_size,
    p->base_color, p->alt_color, p->swing_color, p->clash_color);
}

// --- Accent/Crystal Blade Styles ---

static int BuildAccentPulse(const IniPreset* p, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "pulse %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentBlink(const IniPreset* p, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "blink %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentRandom(const IniPreset* p, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "random %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentStatic(const IniPreset* p, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "static %s", p->base_color);
}

static int BuildAccentColorCycle(const IniPreset* p, char* buf, int buf_size) {
  return BuildStyleString(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->base_color, p->alt_color, p->blast_color, p->lockup_color);
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

#endif // PROPS_STYLE_REGISTRY_H
