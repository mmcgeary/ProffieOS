// ProffieOS/config/ini_config_example.h
//
// Example configuration for the INI Config Prop.
// With this prop, styles, colors, gestures, and buttons are
// configured via saber_config.ini on the SD card.

#ifdef CONFIG_TOP
#include "proffieboard_v3_config.h"

#define NUM_BLADES 1
#define NUM_BUTTONS 2
#define VOLUME 2000
#define CLASH_THRESHOLD_G 8.0
const unsigned int maxLedsPerStrip = 144;

#define ENABLE_AUDIO
#define ENABLE_MOTION
#define ENABLE_WS2811
#define ENABLE_SD
#define ENABLE_SERIAL

#define INI_NUM_BLADES NUM_BLADES
#endif

#ifdef CONFIG_PROP
#include "../props/saber_ini_config.h"
#endif

#ifdef CONFIG_PRESETS

// Minimal preset — INI prop overrides at boot.
Preset presets[] = {
  { "font1", "tracks/track1.wav",
    StylePtr<Blue>(),
    "INI Config"
  },
};

BladeConfig blades[] = {
  { 0, WS281XBladePtr<144, bladePin, Color8::GRB, PowerPINS<bladePowerPin2, bladePowerPin3>>(),
    CONFIGARRAY(presets) },
};

#endif
