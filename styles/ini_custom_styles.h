// ProffieOS/styles/ini_custom_styles.h
#ifndef STYLES_INI_CUSTOM_STYLES_H
#define STYLES_INI_CUSTOM_STYLES_H

#include "ini_style_arg_ids.h"

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

// Individual Base Templates (Core)
using IniAudioFlickerCoreBlade = IniCoreWrapper<AudioFlicker<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>;

using IniHumpFlickerCoreBlade = IniCoreWrapper<HumpFlickerL<RgbArg<BASE_COLOR_ARG,Magenta>,50>>;

using IniPulsingStripesCoreBlade = IniCoreWrapper<StripesX<Int<3000>,Int<-3000>,RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<12000>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,Pulsing<RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<8000>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,1400>>>;

using IniEnergyCoreBlade = IniCoreWrapper<AudioFlicker<BrownNoiseFlicker<RgbArg<ALT_COLOR_ARG,White>,Stripes<5000,-300,Mix<Int<7710>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<25700>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<1285>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>>,300>,Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,White>>>;

using IniFireUnstableCoreBlade = IniCoreWrapper<StaticFire<BrownNoiseFlicker<RgbArg<BASE_COLOR_ARG,Red>,RandomPerLEDFlicker<Mix<Int<3213>,Black,RgbArg<ALT_COLOR_ARG,Orange>>,Mix<Int<7710>,Black,RgbArg<ALT_COLOR2_ARG,Orange>>>,300>,Mix<Int<10280>,Black,RgbArg<ALT_COLOR3_ARG,Yellow>>,0,6,10,1000,2>>;

using IniPlasmaCoreBlade = IniCoreWrapper<StaticFire<Mix<SmoothStep<Int<2000>,Int<-2000>>,Stripes<16000,-3900,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Mix<Int<8172>,StaticFire<Mix<SmoothStep<Int<2000>,Int<-2000>>,Stripes<16000,-3900,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Mix<Int<8172>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Stripes<2500,-3500,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<ALT_COLOR2_ARG,Blue>,Mix<Int<16000>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,RgbArg<ALT_COLOR3_ARG,DodgerBlue>>>,White>,RgbArg<ALT_COLOR2_ARG,Blue>,0,6,1,2000,3>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Mix<Int<16384>,RgbArg<ALT_COLOR_ARG,Black>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,Stripes<2500,-3500,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>,Blue,Mix<Int<16000>,Black,RgbArg<BASE_COLOR_ARG,Rgb<100,100,150>>>,RgbArg<ALT_COLOR3_ARG,DodgerBlue>>>,White>,Blue,0,6,1,2000,3>>;

using IniRainbowCoreBlade = IniCoreWrapper<Rainbow>;

using IniEnergyBladeCoreBlade = IniCoreWrapper<Remap<CenterDistF<>,Stripes<2000,-3000,AudioFlicker<RgbArg<ALT_COLOR_ARG,White>,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<3855>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,AudioFlicker<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>,Rgb<50,50,75>>>>;

using IniLavaCoreBlade = IniCoreWrapper<StripesX<Sin<Int<4>,Int<3000>,Int<6000>>,Scale<TwistAngle<>,Int<-50>,Int<-100>>,StripesX<Sin<Int<3>,Int<1000>,Int<3000>>,Scale<TwistAngle<>,Int<25>,Int<80>>,Pulsing<Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>,Mix<Int<2570>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,3000>,Mix<Sin<Int<2>>,Mix<Int<25700>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<1285>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>>>,Mix<Int<7710>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,Pulsing<Mix<Int<6425>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,StripesX<Sin<Int<2>,Int<2000>,Int<4000>>,Sin<Int<2>,Int<25>,Int<75>>,Mix<Sin<Int<4>>,RgbArg<BASE_COLOR_ARG,Blue>,Mix<Int<6425>,RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>,Mix<Int<12336>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>>,2000>,Pulsing<Mix<Int<16384>,Black,RgbArg<BASE_COLOR_ARG,Blue>>,Mix<Int<6425>,RgbArg<ALT_COLOR2_ARG,Black>,RgbArg<BASE_COLOR_ARG,Blue>>,3000>>>;

using IniSparkleCoreBlade = IniCoreWrapper<Sparkle<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,White>>>;

using IniFireCoreBlade = IniCoreWrapper<StyleFire<RgbArg<BASE_COLOR_ARG,Blue>,RgbArg<ALT_COLOR_ARG,Cyan>>>;

#endif // STYLES_INI_CUSTOM_STYLES_H
