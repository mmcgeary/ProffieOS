#ifdef CONFIG_TOP
#include "proffieboard_config.h"
#define NUM_BLADES 2
#define NUM_BUTTONS 2
#define VOLUME 100
const unsigned int maxLedsPerStrip = 144;
#define CLASH_THRESHOLD_G 3.5
#define ENABLE_AUDIO
#define ENABLE_MOTION
#define ENABLE_WS2811
#define ENABLE_SD
#define NO_REPEAT_RANDOM
#endif

#ifdef CONFIG_PRESETS
Preset presets[] = {
 { "Ezra", "tracks/track1.wav",
    StylePtr<Blue>(),
    StylePtr<Green>(),
    "INI Config"
 }
  };
    BladeConfig blades[] = {
     { 0, WS281XBladePtr<144, bladePin, Color8::GRB, PowerPINS<bladePowerPin2, bladePowerPin3> >(),
        WS281XBladePtr<1, blade2Pin, Color8::GRB, PowerPINS<bladePowerPin4> >()
      , CONFIGARRAY(presets)},
    };
#endif

#ifdef CONFIG_BUTTONS
Button PowerButton(BUTTON_POWER, powerButtonPin, "pow");
    Button AuxButton(BUTTON_AUX, auxPin, "aux");
#endif
