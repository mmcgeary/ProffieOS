# ProffieOS

The open source operating system. Proffie OS is supported on various platforms ranging from Teensy 3.2 development boards to its own dedicated ProffieBoard reference hardware.

Proffie OS supports:
- :fire: SmoothSwing V1/V2 Algorithm
- :fire: NEC styled lightsaber sound fonts (polyphonic)
- :fire: Plecter styled lightsaber sound fonts ( monophonic)
- :fire: Driving Adressable LED strips
- :fire: Driving Segmented LED strips
- :fire: Quad/Tri LED stars.

### Getting started  
* ProffieOS Documentation: https://pod.hubbe.net/
* ProffieOS: https://fredrik.hubbe.net/lightsaber/proffieos.html
* Proffieboard v1.5: https://fredrik.hubbe.net/lightsaber/v4
* Proffieboard v2.2: https://fredrik.hubbe.net/lightsaber/v5
* Proffieboard v3.9: https://fredrik.hubbe.net/lightsaber/v6
* TeensySaber: http://fredrik.hubbe.net/lightsaber/v3/
* Support forum: http://crucible.hubbe.net

---

## The INI Configuration Prop (`saber_ini_config.h`)

This repository includes a modernized, INI-driven prop file architecture (`props/saber_ini_config.h`) designed to dramatically reduce the need for recompiling and flashing the board for configuration changes.

### Key Features

1. **SD Card Configuration (`saber_config.ini`)**:
   Instead of hardcoding styles, colors, and global settings into your board's `.h` config file, these are offloaded to an INI file placed on the root of your SD card.
2. **Dynamic Hardware Settings**:
   Settings that traditionally required a re-flash have been converted into dynamic variables parsed at boot. You can modify these directly in the INI file or via the WebUSB Companion App:
   * **Volume** (`0-3000` scale)
   * **Blade Dimming** (`0-100%`)
   * **Power/Idle Timers** (`idle_off_time`, `motion_timeout`)
   * **Button Timings** (`button_short_click_timeout`, `button_double_click_timeout`)
   * **Gestures** (Twist, Stab, Swing, Thrust, Force Push, Melt toggles)
3. **Custom Styles Architecture**:
   The `styles/ini_custom_styles.h` file acts as a repository of highly parameterized style templates. The INI file maps predefined templates (like `standard`, `audio_flicker`, `unstable`) to specific colors, animation speeds, and sizes without ever writing C++ code.
4. **Companion App Integration**:
   A dedicated [ProffieOS-Companion WebUSB App](https://github.com/proffieboard/ProffieOS-Companion) connects to this prop. The prop responds to custom serial commands (`READ_INI`, `WRITE_INI_BANK`, `GET_HW_PROFILE`) to allow for seamless GUI-based editing, style previewing, and bidirectional config syncing.

### How to use

1. Enable the INI prop by including `#include "../props/saber_ini_config.h"` in your board config's `CONFIG_PROP` section.
2. Ensure you have the necessary dynamic `#define` flags enabled in your `CONFIG_TOP` (e.g., `DYNAMIC_BLADE_DIMMING`, `DYNAMIC_BLADE_LENGTH`). See `config/ini_config_example.h` for a complete example.
3. Place `props/saber_config_example.ini` onto the root of your SD card and rename it to `saber_config.ini`.
4. Connect to the Companion App to manage your lightsaber!