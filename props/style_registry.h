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

// --- Main Blade Style Build Functions ---

static int BuildStandard(const IniPreset* p, char* buf, int buf_size) {
  // Subtle flicker baseline using parser-supported "cycle".
  // cycle <start> <base> <flicker> <blast> <lockup>
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->base_color, p->alt_color, p->blast_color, p->lockup_color);
}

static int BuildUnstable(const IniPreset* p, char* buf, int buf_size) {
  // style_parser: unstable <warm> <warmer> <hot> <sparks> <ignition_ms> <retraction_ms>
  return snprintf(buf, buf_size, "unstable %s %s %s %s %u %u",
    p->base_color, p->alt_color, p->blast_color, p->clash_color,
    p->ignition_time, p->retraction_time);
}

static int BuildFire(const IniPreset* p, char* buf, int buf_size) {
  // style_parser: fire <warm_color> <hot_color>
  return snprintf(buf, buf_size, "fire %s %s", p->base_color, p->alt_color);
}

static int BuildRainbow(const IniPreset* p, char* buf, int buf_size) {
  // style_parser: rainbow <ignition_ms> <retraction_ms>
  return snprintf(buf, buf_size, "rainbow %u %u", p->ignition_time, p->retraction_time);
}

static int BuildStrobe(const IniPreset* p, char* buf, int buf_size) {
  // style_parser: strobe <standby_color> <flash_color> <freq> <ms> <ignition_ms> <retraction_ms>
  return snprintf(buf, buf_size, "strobe %s %s 15 1 %u %u",
    p->base_color, p->alt_color, p->ignition_time, p->retraction_time);
}

static int BuildPulse(const IniPreset* p, char* buf, int buf_size) {
  // Stronger breathing/flicker than standard.
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->alt_color, p->alt_color, p->blast_color, p->lockup_color);
}

static int BuildRotoscope(const IniPreset* p, char* buf, int buf_size) {
  // Film-like shimmer using low-frequency strobe.
  return snprintf(buf, buf_size, "strobe %s %s 8 2 %u %u",
    p->base_color, p->alt_color, p->ignition_time, p->retraction_time);
}

static int BuildGhostly(const IniPreset* p, char* buf, int buf_size) {
  // Pale, drifting ghost flicker.
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->alt_color, p->clash_color, p->blast_color, p->lockup_color);
}

static int BuildLightning(const IniPreset* p, char* buf, int buf_size) {
  // Sharp electrical flash behavior.
  return snprintf(buf, buf_size, "strobe %s %s 25 2 %u %u",
    p->base_color, p->alt_color, p->ignition_time, p->retraction_time);
}

static int BuildDarksaber(const IniPreset* p, char* buf, int buf_size) {
  // Alias to supported "unstable"
  return BuildUnstable(p, buf, buf_size);
}

static int BuildKylo(const IniPreset* p, char* buf, int buf_size) {
  // Alias to supported "unstable"
  return BuildUnstable(p, buf, buf_size);
}

static int BuildPrequels(const IniPreset* p, char* buf, int buf_size) {
  // Smooth but still visibly alive.
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->base_color, p->alt_color, p->blast_color, p->lockup_color);
}

static int BuildSequels(const IniPreset* p, char* buf, int buf_size) {
  // Slightly rougher modern flicker.
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->alt_color, p->clash_color, p->blast_color, p->lockup_color);
}

static int BuildAncient(const IniPreset* p, char* buf, int buf_size) {
  // Aged, uneven glow profile.
  return snprintf(buf, buf_size, "cycle %s %s %s %s %s",
    p->base_color, p->alt_color, p->swing_color, p->blast_color, p->lockup_color);
}

static int BuildStaticColor(const IniPreset* p, char* buf, int buf_size) {
  // Explicit non-flicker fallback.
  return snprintf(buf, buf_size, "standard %s %s %u %u",
    p->base_color, p->clash_color, p->ignition_time, p->retraction_time);
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
