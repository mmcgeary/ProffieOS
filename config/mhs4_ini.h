#ifdef CONFIG_TOP
#include "proffieboard_v3_config.h"
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
#include "../styles/ini_custom_styles.h"

Preset presets[] = {
  { "Style0", "tracks/track1.wav", StylePtr<IniAudioFlickerCoreBlade>() },
  { "Style1", "tracks/track1.wav", StylePtr<IniHumpFlickerCoreBlade>() },
  { "Style2", "tracks/track1.wav", StylePtr<IniPulsingStripesCoreBlade>() },
  { "Style3", "tracks/track1.wav", StylePtr<IniEnergyCoreBlade>() },
  { "Style4", "tracks/track1.wav", StylePtr<IniFireUnstableCoreBlade>() },
  { "Style5", "tracks/track1.wav", StylePtr<IniPlasmaCoreBlade>() },
  { "Style6", "tracks/track1.wav", StylePtr<IniRainbowCoreBlade>() },
  { "Style7", "tracks/track1.wav", StylePtr<IniEnergyBladeCoreBlade>() },
  { "Style8", "tracks/track1.wav", StylePtr<IniLavaCoreBlade>() },
  { "Style9", "tracks/track1.wav", StylePtr<IniSparkleCoreBlade>() },
  { "Style10", "tracks/track1.wav", StylePtr<IniFireCoreBlade>() },
};

BladeConfig blades[] = {
 { 0, WS281XBladePtr<126, bladePin, Color8::GRB, PowerPINS<bladePowerPin2, bladePowerPin3> >(),
   CONFIGARRAY(presets), "blade" },
 { NO_BLADE, WS281XBladePtr<1, blade2Pin, Color8::GRB, PowerPINS<bladePowerPin4, bladePowerPin5> >(),
   CONFIGARRAY(presets), "noblade" },
};
#endif

#ifdef CONFIG_BUTTONS
Button PowerButton(BUTTON_POWER, auxPin, "pow");
#endif
