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
#define INI_NUM_BLADES NUM_BLADES
#endif

#ifdef CONFIG_PROP
#include "../props/saber_ini_config.h"
#endif

#ifdef CONFIG_PRESETS
#include "../styles/ini_custom_styles.h"

Preset presets[] = {
  { "Anakin", "tracks/track1.wav", StylePtr<IniAudioFlickerCoreBlade>(),StylePtr<IniAudioFlickerCoreBlade>() },
  { "Ezra", "tracks/track1.wav", StylePtr<IniHumpFlickerCoreBlade>(), StylePtr<IniHumpFlickerCoreBlade>()},
  { "Exile", "tracks/track1.wav", StylePtr<IniFilmCoreBlade>(), StylePtr<IniBlinkingCoreBlade>() },
  { "Kestis", "tracks/track1.wav", StylePtr<IniPulsingStripesCoreBlade>(),StylePtr<IniPulsingStripesCoreBlade>() },
  { "DarkSbr", "tracks/track1.wav", StylePtr<IniEnergyCoreBlade>(),StylePtr<IniEnergyCoreBlade>() },
  { "Kylo", "tracks/track1.wav", StylePtr<IniFireUnstableCoreBlade>(),StylePtr<IniFireUnstableCoreBlade>() },
  { "Grevious", "tracks/track1.wav", StylePtr<IniPlasmaCoreBlade>(),StylePtr<IniPlasmaCoreBlade>() },
  { "Juhani", "tracks/track1.wav", StylePtr<IniRainbowCoreBlade>(),StylePtr<IniRainbowCoreBlade>() },
  { "Kanan", "tracks/track1.wav", StylePtr<IniEnergyBladeCoreBlade>(),StylePtr<IniEnergyBladeCoreBlade>() },
  { "Jolee", "tracks/track1.wav", StylePtr<IniLavaCoreBlade>(),StylePtr<IniLavaCoreBlade>() },
  { "Handmaiden", "tracks/track1.wav", StylePtr<IniSparkleCoreBlade>(),StylePtr<IniSparkleCoreBlade>() },
  { "Malak", "tracks/track1.wav", StylePtr<IniFireCoreBlade>(),StylePtr<IniFireCoreBlade>() },
  { "Emperor", "tracks/track1.wav", StylePtr<IniBlinkingCoreBlade>(),StylePtr<IniBlinkingCoreBlade>() },
  { "Huntress", "tracks/track1.wav", StylePtr<IniColorCycleCoreBlade>(),StylePtr<IniColorCycleCoreBlade>()  },
};
    BladeConfig blades[] = {
     { 0, WS281XBladePtr<144, bladePin, Color8::GRB, PowerPINS<bladePowerPin2, bladePowerPin3> >(),
        WS281XBladePtr<1, blade2Pin, Color8::GRB, PowerPINS<bladePowerPin4> >()
      , CONFIGARRAY(presets) },
    };
#endif

#ifdef CONFIG_BUTTONS
Button PowerButton(BUTTON_POWER, powerButtonPin, "pow");
    Button AuxButton(BUTTON_AUX, auxPin, "aux");
#endif
