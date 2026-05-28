// ProffieOS/styles/ini_custom_styles.h
#ifndef STYLES_INI_CUSTOM_STYLES_H
#define STYLES_INI_CUSTOM_STYLES_H

#include "ini_style_arg_ids.h"
#include "blinking.h"
#include "color_cycle.h"

// Core Wrapper Template (LTO will deduplicate identical instances)
template<class BASE>
using IniCoreWrapper = Layers<
  BASE,
  ResponsiveLockupL<RgbArg<LOCKUP_COLOR_ARG,White>,TrInstant,TrFadeX<IntArg<LOCKUP_FADE_ARG,100>>,IntArg<LOCKUP_SIZE_ARG,26000>>,
  ResponsiveLightningBlockL<RgbArg<LB_COLOR_ARG,White>>,
  ResponsiveMeltL<Mix<TwistAngle<>,RgbArg<MELT_BASE_ARG,Red>,RgbArg<MELT_ALT_ARG,Yellow>>>,
  ResponsiveDragL<RgbArg<DRAG_COLOR_ARG,White>>,
  ResponsiveClashL<RgbArg<CLASH_COLOR_ARG,White>,TrInstant,TrFadeX<IntArg<CLASH_FADE_ARG,200>>,IntArg<LOCKUP_SIZE_ARG,26000>>,
  ResponsiveBlastL<RgbArg<BLAST_COLOR_ARG,White>>,
  ResponsiveBlastWaveL<RgbArg<BLAST_COLOR_ARG,White>>,
  ResponsiveBlastFadeL<RgbArg<BLAST_COLOR_ARG,White>>,
  ResponsiveStabL<RgbArg<STAB_COLOR_ARG,White>>,
  InOutTrL<TrWipeX<IntArg<IGNITION_TIME_ARG,300>>,TrWipeInX<IntArg<RETRACTION_TIME_ARG,500>>>
>;

// ── Base Style 0: AudioFlicker ────────────────────────────────────────────────
// BASE_COLOR, ALT_COLOR are the two mix endpoints driven by sound level.
// No speed params — entirely controlled by audio amplitude.
using IniAudioFlickerCoreBlade = IniCoreWrapper<AudioFlicker<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>;

// ── Base Style 1: HumpFlicker ─────────────────────────────────────────────────
// HUMP_WIDTH_ARG: controls width of the wandering brightness hump (default 50)
using IniHumpFlickerCoreBlade = IniCoreWrapper<HumpFlickerLX<RgbArg<BASE_COLOR_ARG,Magenta>,IntArg<HUMP_WIDTH_ARG,50>>>;

// ── Base Style 2: PulsingStripes ──────────────────────────────────────────────
// STRIPE_WIDTH_ARG: stripe width (default 3000)
// STRIPE_SPEED_ARG: stripe scroll speed, negative = reverse (default -3000)
// PULSE_SPEED_ARG:  time in ms for one full BASE->darkened->BASE pulse (default 1400)
using IniPulsingStripesCoreBlade = IniCoreWrapper<StripesX<IntArg<STRIPE_WIDTH_ARG,3000>,IntArg<STRIPE_SPEED_ARG,-3000>,RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<12000>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,PulsingX<RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<8000>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,IntArg<PULSE_SPEED_ARG,1400>>>>;

// ── Base Style 3: Energy ──────────────────────────────────────────────────────
// BASE_COLOR: main energy blade colour (e.g. Rgb<100,100,150>)
// ALT_COLOR:  brightest highlight / spark colour (e.g. White)
// ALT_COLOR2: shadow / depth colour that mixes with base (e.g. Black)
// Note: inner Stripes params are hardcoded — nesting inside BrownNoiseFlicker
// makes StripesX unsafe here; colors are still fully dynamic via RgbArg.
using IniEnergyCoreBlade = IniCoreWrapper<AudioFlicker<BrownNoiseFlicker<RgbArg<ALT_COLOR_ARG,White>,Stripes<5000,-300,Mix<Int<7710>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<25700>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<1285>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>>,300>,Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,White>>>;

// ── Base Style 4: FireUnstable ────────────────────────────────────────────────
// BASE_COLOR: fire core colour (e.g. Red)
// ALT_COLOR:  mid-flame colour (e.g. Orange)
// ALT_COLOR2: secondary flicker colour (e.g. Orange)
// ALT_COLOR3: tip colour (e.g. Yellow)
// Fire sim params (DELAY, SPEED, COOLING) must be hard-coded compile-time constants.
using IniFireUnstableCoreBlade = IniCoreWrapper<StaticFire<BrownNoiseFlicker<RgbArg<BASE_COLOR_ARG,Red>,RandomPerLEDFlicker<Mix<Int<3213>,Black,RgbArg<ALT_COLOR_ARG,Orange>>,Mix<Int<7710>,Black,RgbArg<ALT_COLOR2_ARG,Orange>>>,300>,Mix<Int<10280>,Black,RgbArg<ALT_COLOR3_ARG,Yellow>>,0,6,10,1000,2>>;

// ── Base Style 5: Plasma ──────────────────────────────────────────────────────
// BASE_COLOR: plasma core colour (e.g. Rgb<100,100,150>)
// ALT_COLOR:  dark/shadow mix (e.g. Black)
// ALT_COLOR2: secondary plasma accent (e.g. Blue)
// ALT_COLOR3: outer glow colour (e.g. DodgerBlue)
// Note: inner Stripes parameters are hardcoded — the nesting depth makes
// StripesX unsafe here; colors are still fully dynamic via RgbArg.
using IniPlasmaCoreBlade = IniCoreWrapper<StaticFire<Mix<SmoothStep<Int<2000>,Int<-2000>>,Stripes<16000,-3900,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Mix<Int<8172>,StaticFire<Mix<SmoothStep<Int<2000>,Int<-2000>>,Stripes<16000,-3900,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Mix<Int<8172>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Stripes<2500,-3500,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<ALT_COLOR2_ARG,Blue>,Mix<Int<16000>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,RgbArg<ALT_COLOR3_ARG,DodgerBlue>>>,White>,RgbArg<ALT_COLOR2_ARG,Blue>,0,6,1,2000,3>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Stripes<2500,-3500,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Blue,Mix<Int<16000>,Black,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,RgbArg<ALT_COLOR3_ARG,DodgerBlue>>>,White>,Blue,0,6,1,2000,3>>;

// ── Base Style 6: Rainbow ─────────────────────────────────────────────────────
// No argifiable params — the hue cycle rate is baked into the sin_table math.
using IniRainbowCoreBlade = IniCoreWrapper<Rainbow>;

// ── Base Style 7: EnergyBlade ─────────────────────────────────────────────────
// BASE_COLOR: main blade colour (e.g. Blue)
// ALT_COLOR:  bright highlight (e.g. White)
// ALT_COLOR2: dark shadow mix colour (e.g. Black)
// STRIPE_WIDTH_ARG / STRIPE_SPEED_ARG: control the scroll
using IniEnergyBladeCoreBlade = IniCoreWrapper<Remap<CenterDistF<>,StripesX<IntArg<STRIPE_WIDTH_ARG,2000>,IntArg<STRIPE_SPEED_ARG,-3000>,AudioFlicker<RgbArg<ALT_COLOR_ARG,White>,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<3855>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,AudioFlicker<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>,Rgb<50,50,75>>>>;

// ── Base Style 8: Lava ────────────────────────────────────────────────────────
// BASE_COLOR:  lava base colour (e.g. Blue/orange/red as desired)
// ALT_COLOR:   bright highlight mix end (e.g. White)
// ALT_COLOR2:  dark/depth colour (e.g. Black)
// PULSE_SPEED_ARG: pulsing oscillation period ms (default 2000–3000)
using IniLavaCoreBlade = IniCoreWrapper<StripesX<Sin<Int<4>,Int<3000>,Int<6000>>,Scale<TwistAngle<>,Int<-50>,Int<-100>>,StripesX<Sin<Int<3>,Int<1000>,Int<3000>>,Scale<TwistAngle<>,Int<25>,Int<80>>,PulsingX<Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>,Mix<Int<2570>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,IntArg<PULSE_SPEED_ARG,3000>>,Mix<Sin<Int<2>>,Mix<Int<25700>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<1285>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>>>,Mix<Int<7710>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,PulsingX<Mix<Int<6425>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,StripesX<Sin<Int<2>,Int<2000>,Int<4000>>,Sin<Int<2>,Int<25>,Int<75>>,Mix<Sin<Int<4>>,RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>,Mix<Int<12336>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>>,IntArg<PULSE_SPEED_ARG,2000>>,PulsingX<Mix<Int<16384>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<6425>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,IntArg<PULSE_SPEED_ARG,3000>>>>;

// ── Base Style 9: Sparkle ─────────────────────────────────────────────────────
// BASE_COLOR: background colour
// ALT_COLOR:  sparkle highlight colour
using IniSparkleCoreBlade = IniCoreWrapper<Sparkle<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>;

// ── Base Style 10: Fire (simple) ──────────────────────────────────────────────
// BASE_COLOR: fire bottom/root colour
// ALT_COLOR:  fire tip colour
using IniFireCoreBlade = IniCoreWrapper<StyleFire<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,Cyan>>>;

// ── Base Style 11: Blinking ───────────────────────────────────────────────────
// BASE_COLOR: on-phase colour
// ALT_COLOR:  off-phase colour (typically Black)
// FLICKER_SPEED_ARG: total blink period in ms (default 1000)
// FLICKER_DEPTH_ARG: on-duty promille 0-1000; 500 = 50% on (default 500)
using IniBlinkingCoreBlade = IniCoreWrapper<BlinkingX<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,Black>,IntArg<FLICKER_SPEED_ARG,1000>,IntArg<FLICKER_DEPTH_ARG,500>>>;

// ── Base Style 12: ColorCycle ─────────────────────────────────────────────────
// BASE_COLOR:       off-state cycle colour
// ALT_COLOR:        on-state cycle colour
// ALT_COLOR2:       base (unlit pixel) colour (default Black)
// CYCLE_OFF_PCT_ARG: lit-section size when off, percent 0-100 (default 0)
// CYCLE_OFF_RPM_ARG: rotation speed when off (default 1)
// CYCLE_ON_PCT_ARG:  lit-section size when on, percent 0-100 (default 30)
// CYCLE_ON_RPM_ARG:  rotation speed when on (default 600)
// CYCLE_FADE_ARG:    on<->off transition time ms (default 1000)
using IniColorCycleCoreBlade = IniCoreWrapper<ColorCycleX<RgbArg<BASE_COLOR_ARG,Blue>,IntArg<CYCLE_OFF_PCT_ARG,0>,IntArg<CYCLE_OFF_RPM_ARG,1>,RgbArg<ALT_COLOR_ARG,Cyan>,IntArg<CYCLE_ON_PCT_ARG,30>,IntArg<CYCLE_ON_RPM_ARG,600>,IntArg<CYCLE_FADE_ARG,1000>,RgbArg<ALT_COLOR2_ARG,Black>>>;

// ── Base Style 13: Film ───────────────────────────────────────────────────────
// BASE_COLOR: main blade colour
using IniFilmCoreBlade = Layers<AudioFlicker<Stripes<8000,-2500,RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<16000>,Black,RgbArg<BASE_COLOR_ARG,Blue>>>,RgbArg<BASE_COLOR_ARG,Blue>>, ResponsiveLockupL<RgbArg<LOCKUP_COLOR_ARG,White>,TrInstant,TrFade<100>,Int<26000>>, ResponsiveLightningBlockL<RgbArg<LB_COLOR_ARG,White>>, ResponsiveMeltL<Mix<TwistAngle<>,Red,Yellow>>, ResponsiveDragL<RgbArg<DRAG_COLOR_ARG,White>>, ResponsiveClashL<RgbArg<CLASH_COLOR_ARG,White>,TrInstant,TrFade<200>,Int<26000>>, ResponsiveBlastL<RgbArg<BLAST_COLOR_ARG,White>>, ResponsiveBlastWaveL<RgbArg<BLAST_COLOR_ARG,White>>, ResponsiveBlastFadeL<RgbArg<BLAST_COLOR_ARG,White>>, ResponsiveStabL<RgbArg<STAB_COLOR_ARG,White>>, InOutTrL<TrWipeX<IntArg<IGNITION_TIME_ARG, 300>>,TrWipeInX<IntArg<RETRACTION_TIME_ARG, 500>>>>;

#endif // STYLES_INI_CUSTOM_STYLES_H
