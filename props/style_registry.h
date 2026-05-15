// ProffieOS/props/style_registry.h
#ifndef PROPS_STYLE_REGISTRY_H
#define PROPS_STYLE_REGISTRY_H

#include "runtime_config.h"

#define MAX_STYLE_STRING_LEN 256

struct IniPreset;

typedef int (*StyleBuildFn)(const IniPreset* preset, char* buf, int buf_size);

struct IniStyleEntry {
  const char* name;
  const char* description;
  StyleBuildFn build;
};

// Helper: write common effect args shared by ALL main blade styles
static int WriteCommonEffectArgs(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, " %s %s %s %s %s %s",
    p->blast_color, p->clash_color, p->lockup_color,
    p->drag_color, p->lb_color, p->stab_color);
}

// Helper: write timing args
static int WriteTimingArgs(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, " %u %u", p->ignition_time, p->retraction_time);
}

// --- Main Blade Style Build Functions ---

static int BuildStandard(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "standard %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildUnstable(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "unstable %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildFire(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "fire %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildRainbow(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "rainbow %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildStrobe(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "strobe %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildPulse(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "pulse %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildRotoscope(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "rotoscope %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildGhostly(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "ghostly %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildLightning(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "lightning %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildDarksaber(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "darksaber %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildKylo(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "kylo %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildPrequels(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "prequels %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildSequels(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "sequels %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildAncient(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "ancient %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

static int BuildStaticColor(const IniPreset* p, char* buf, int buf_size) {
  int n = snprintf(buf, buf_size, "static %s %s %u",
    p->base_color, p->alt_color, p->ignition_time);
  n += WriteCommonEffectArgs(p, buf + n, buf_size - n);
  n += snprintf(buf + n, buf_size - n, " %u", p->retraction_time);
  return n;
}

// --- Accent/Crystal Blade Styles ---

static int BuildAccentPulse(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, "pulse %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentBlink(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, "blink %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentRandom(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, "random %s %u", p->base_color, p->accent_speed);
}

static int BuildAccentStatic(const IniPreset* p, char* buf, int buf_size) {
  return snprintf(buf, buf_size, "static %s", p->base_color);
}

// --- Registry Tables ---

const IniStyleEntry ini_style_registry[] = {
  {"standard",    "Classic solid blade",           BuildStandard},
  {"unstable",    "Flickering unstable blade",     BuildUnstable},
  {"fire",        "Flame animated blade",          BuildFire},
  {"rainbow",     "Cycling rainbow colors",        BuildRainbow},
  {"strobe",      "Strobing flicker blade",        BuildStrobe},
  {"pulse",       "Pulsing glow blade",            BuildPulse},
  {"rotoscope",   "Original trilogy look",         BuildRotoscope},
  {"ghostly",     "Transparent ethereal blade",    BuildGhostly},
  {"lightning",   "Lightning animated blade",      BuildLightning},
  {"darksaber",   "Darksaber white-core style",    BuildDarksaber},
  {"kylo",        "Crossguard unstable variant",   BuildKylo},
  {"prequels",    "Prequel-era smooth blade",      BuildPrequels},
  {"sequels",     "Sequel-era slight flicker",     BuildSequels},
  {"ancient",     "Ancient Jedi temple style",     BuildAncient},
  {"static",      "Completely static blade",       BuildStaticColor},
};

const int INI_STYLE_COUNT = sizeof(ini_style_registry) / sizeof(ini_style_registry[0]);

const IniStyleEntry ini_accent_registry[] = {
  {"pulse",   "Slow pulsing",       BuildAccentPulse},
  {"blink",   "Blinking",           BuildAccentBlink},
  {"random",  "Random flickering",  BuildAccentRandom},
  {"static",  "Always-on static",   BuildAccentStatic},
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
