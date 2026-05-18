#ifdef CONFIG_TOP
#include "proffieboard_config.h"
#define NUM_BLADES 1
#define NUM_BUTTONS 1
#define VOLUME 100
const unsigned int maxLedsPerStrip = 144;
#define CLASH_THRESHOLD_G 3.5
#define ENABLE_AUDIO
#define ENABLE_MOTION
#define ENABLE_WS2811
#define ENABLE_SD
#define NO_REPEAT_RANDOM
#define ORIENTATION ORIENTATION_USB_TOWARDS_BLADE
#define BLADE_DETECT_PIN powerButtonPin
#define INI_NUM_BLADES NUM_BLADES
#endif

#ifdef CONFIG_PROP
#include "../props/saber_ini_config.h"
#endif

#ifdef CONFIG_PRESETS
Preset blade[] = {
 { "Kestis", "tracks/track1.wav",
    StylePtr<Blue>(),
    "INI Config"
 }
  };
Preset noblade[] = {
   { "Calibrate", "tracks/Force_Theme.wav",
    StylePtr<Blue>(),
    "INI Config"}
    };
BladeConfig blades[] = {
 { 0, WS281XBladePtr<126, bladePin, Color8::GRB, PowerPINS<bladePowerPin2, bladePowerPin3> >()
  , CONFIGARRAY(blade), "blade" },
 {
   NO_BLADE, WS281XBladePtr<1, blade2Pin, Color8::GRB, PowerPINS<bladePowerPin4, bladePowerPin5> >()
   , CONFIGARRAY(noblade), "noblade" },
};
#endif

#ifdef CONFIG_BUTTONS
Button PowerButton(BUTTON_POWER, auxPin, "pow");
#endif
