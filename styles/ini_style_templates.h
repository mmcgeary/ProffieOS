#ifndef STYLES_INI_STYLE_TEMPLATES_H
#define STYLES_INI_STYLE_TEMPLATES_H

#include "ini_style_arg_ids.h"
#include "../functions/center_dist.h"
#include "../functions/int_arg.h"
#include "../functions/mult.h"
#include "../functions/scale.h"
#include "../functions/sin.h"
#include "../functions/twist_angle.h"
#include "../transitions/fade.h"
#include "../transitions/wipe.h"
#include "audio_flicker.h"
#include "blinking.h"
#include "brown_noise_flicker.h"
#include "color_cycle.h"
#include "color_select.h"
#include "fire.h"
#include "hump_flicker.h"
#include "inout_helper.h"
#include "lockup.h"
#include "mix.h"
#include "pulsing.h"
#include "rainbow.h"
#include "random_blink.h"
#include "random_flicker.h"
#include "random_per_led_flicker.h"
#include "remap.h"
#include "responsive_styles.h"
#include "rgb_arg.h"
#include "rotate_color.h"
#include "sparkle.h"
#include "strobe.h"
#include "stripes.h"
#include "style_ptr.h"

namespace ini_args = ini_style_args;

template<class BASE>
using IniPrimaryBlade = InOutTr<
    Layers<
        BASE,
        AlphaL<
            Mix<IntArg<ini_args::kBaseContrastArg, 22000>, Black, RgbArg<ini_args::kAltColorArg, White>>,
            Mult<
                PulsingF<IntArg<ini_args::kFlickerSpeedArg, 60>>,
                IntArg<ini_args::kFlickerDepthArg, 12000>>>,
        AlphaL<
            RotateColorsX<IntArg<ini_args::kDriftRateArg, 600>, RgbArg<ini_args::kBaseColorArg, CYAN>>,
            IntArg<ini_args::kNoiseMixArg, 8000>>,
        AlphaL<RgbArg<ini_args::kStabColorArg, Rgb<255, 68, 0>>, IntArg<ini_args::kWarmShiftArg, 2000>>,
        ResponsiveBlastL<RgbArg<ini_args::kBlastColorArg, White>>,
        ResponsiveClashL<RgbArg<ini_args::kClashColorArg, White>>,
        LockupTrL<
            AlphaL<
                AudioFlicker<
                    RgbArg<ini_args::kLockupColorArg, White>,
                    Mix<
                        IntArg<ini_args::kSparkMixArg, 12288>,
                        Black,
                        RgbArg<ini_args::kLockupColorArg, White>>>,
                Bump<
                    Int<16000>,
                    Scale<SwingSpeed<100>, Int<2000>, IntArg<ini_args::kMotionGainArg, 12000>>>>,
            TrInstant,
            TrFade<300>,
            SaberBase::LOCKUP_NORMAL>,
        ResponsiveLightningBlockL<
            StrobeX<
                RgbArg<ini_args::kLbColorArg, White>,
                AudioFlicker<RgbArg<ini_args::kLbColorArg, White>, Blue>,
                IntArg<ini_args::kJitterAmountArg, 50>,
                Int<1>>>,
        LockupTrL<
            AlphaL<
                RandomPerLEDFlickerL<RgbArg<ini_args::kDragColorArg, White>>,
                SmoothStep<
                    Int<28000>,
                    Scale<IntArg<ini_args::kMotionGainArg, 4096>, Int<1200>, Int<5000>>>>,
            TrWipeIn<180>,
            TrFade<220>,
            SaberBase::LOCKUP_DRAG>,
        LockupTrL<
            AlphaL<
                HumpFlicker<
                    RgbArg<ini_args::kStabColorArg, Rgb<255, 68, 0>>,
                    RotateColorsX<
                        IntArg<ini_args::kDriftRateArg, 3000>,
                        RgbArg<ini_args::kStabColorArg, Rgb<255, 68, 0>>>,
                    90>,
                SmoothStep<
                    Int<28000>,
                    Scale<IntArg<ini_args::kMotionGainArg, 4096>, Int<1000>, Int<4200>>>>,
            TrWipeIn<120>,
            TrWipe<200>,
            SaberBase::LOCKUP_MELT>>,
    TrWipeSparkTipX<
        RgbArg<ini_args::kEmitterColorArg, White>,
        IntArg<ini_args::kIgnitionTimeArg, 300>>,
    TrWipeInSparkTipX<
        RgbArg<ini_args::kEmitterColorArg, White>,
        IntArg<ini_args::kRetractionTimeArg, 800>>,
    ColorSelect<
        IntArg<ini_args::kOffModeArg, 1>,
        TrInstant,
        RgbArg<ini_args::kOffColorArg, Black>,
        PulsingX<
            RgbArg<ini_args::kOffColorArg, Black>,
            Mix<Int<16384>, Black, RgbArg<ini_args::kOffColorArg, Black>>,
            IntArg<ini_args::kOffRateMsArg, 2200>>,
        RandomBlinkX<
            IntArg<ini_args::kOffRateMsArg, 2200>,
            RgbArg<ini_args::kOffColorArg, Black>,
            Black>>>;

using IniBaseStandard = RgbArg<ini_args::kBaseColorArg, CYAN>;
using IniBaseHumpFlicker =
    HumpFlicker<
        RgbArg<ini_args::kBaseColorArg, CYAN>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 24576>,
            Black,
            RgbArg<ini_args::kAltColorArg, WHITE>>,
        100>;
using IniBaseUnstable =
    Layers<
        RgbArg<ini_args::kBaseColorArg, Rgb<150, 0, 0>>,
        BrownNoiseFlickerL<RgbArg<ini_args::kAltColorArg, Red>, IntArg<ini_args::kNoiseMixArg, 120>>,
        StrobeL<
            RgbArg<ini_args::kArg3, Rgb<255, 40, 0>>,
            IntArg<ini_args::kJitterAmountArg, 45>,
            Int<3>>,
        AlphaL<
            RandomFlicker<RgbArg<ini_args::kArg3, Rgb<255, 40, 0>>, Black>,
            IntArg<ini_args::kSparkMixArg, 5000>>>;
using IniBaseFire =
    Layers<
        StaticFire<
            RgbArg<ini_args::kBaseColorArg, RED>,
            RgbArg<ini_args::kAltColorArg, YELLOW>,
            0,
            2,
            0,
            1800,
            6>,
        BrownNoiseFlickerL<
            RgbArg<ini_args::kAltColorArg, YELLOW>,
            IntArg<ini_args::kHeatRandArg, 4500>>,
        AlphaL<
            RgbArg<ini_args::kArg3, Rgb<255, 120, 0>>,
            BrownNoiseF<IntArg<ini_args::kFireCoolingArg, 55>>>>;
using IniBaseRainbow =
    StripesX<Int<1800>, IntArg<ini_args::kRainbowSpeedArg, 800>, Red, Yellow, Green, Cyan, Blue, Magenta>;
using IniBaseStrobe =
    StrobeX<
        RgbArg<ini_args::kBaseColorArg, BLACK>,
        RgbArg<ini_args::kAltColorArg, WHITE>,
        IntArg<ini_args::kArg3, 15>,
        IntArg<ini_args::kArg4, 1>>;
using IniBasePulse =
    Layers<
        RgbArg<ini_args::kBaseColorArg, CYAN>,
        AlphaL<
            RgbArg<ini_args::kAltColorArg, WHITE>,
            Mult<
                PulsingF<IntArg<ini_args::kArg3, 43>>,
                Mult<IntArg<ini_args::kArg4, 9000>, Int<16384>>>>>;
using IniBaseRotoscope =
    HumpFlicker<
        RgbArg<ini_args::kBaseColorArg, Rgb<200, 200, 255>>,
        StripesX<
            IntArg<ini_args::kStripeWidthArg, 35000>,
            IntArg<ini_args::kStripeSpeedArg, 200>,
            Mix<
                IntArg<ini_args::kBaseContrastArg, 14000>,
                Black,
                RgbArg<ini_args::kBaseColorArg, Rgb<200, 200, 255>>>,
            RgbArg<ini_args::kBaseColorArg, Rgb<200, 200, 255>>,
            Mix<
                IntArg<ini_args::kBaseContrastArg, 26000>,
                Black,
                RgbArg<ini_args::kBaseColorArg, Rgb<200, 200, 255>>>>,
        100>;
using IniBaseGhostly =
    Layers<
        Mix<
            IntArg<ini_args::kBaseContrastArg, 20000>,
            Black,
            RgbArg<ini_args::kBaseColorArg, Rgb<200, 255, 255>>>,
        AlphaL<
            AudioFlicker<RgbArg<ini_args::kAltColorArg, Rgb<240, 255, 255>>, Black>,
            IntArg<ini_args::kFlickerDepthArg, 12000>>,
        AlphaL<
            RotateColorsX<
                IntArg<ini_args::kDriftRateArg, 1200>,
                RgbArg<ini_args::kBaseColorArg, Rgb<200, 255, 255>>>,
            IntArg<ini_args::kNoiseMixArg, 6000>>>;
using IniBaseLightning =
    StrobeX<
        Layers<
            RgbArg<ini_args::kBaseColorArg, Blue>,
            BrownNoiseFlickerL<RgbArg<ini_args::kAltColorArg, White>, IntArg<ini_args::kNoiseMixArg, 120>>>,
        RgbArg<ini_args::kArg3, White>,
        IntArg<ini_args::kArg4, 25>,
        IntArg<ini_args::kJitterAmountArg, 2>>;
using IniBaseDarksaber =
    StripesX<
        IntArg<ini_args::kStripeWidthArg, 2800>,
        IntArg<ini_args::kStripeSpeedArg, 11000>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 26000>,
            Black,
            RgbArg<ini_args::kBaseColorArg, White>>,
        RgbArg<ini_args::kAltColorArg, White>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 8000>,
            Black,
            RgbArg<ini_args::kBaseColorArg, White>>>;
using IniBaseKylo =
    Layers<
        RgbArg<ini_args::kBaseColorArg, Rgb<170, 0, 0>>,
        BrownNoiseFlickerL<RgbArg<ini_args::kAltColorArg, Red>, IntArg<ini_args::kNoiseMixArg, 150>>,
        AlphaL<RgbArg<ini_args::kAltColorArg, Red>, IntArg<ini_args::kFlickerDepthArg, 12000>>,
        AlphaL<
            RandomFlicker<RgbArg<ini_args::kArg3, Rgb<255, 180, 0>>, Black>,
            IntArg<ini_args::kSparkMixArg, 5000>>>;
using IniBasePrequels =
    AudioFlicker<
        RgbArg<ini_args::kBaseColorArg, Blue>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 4096>,
            RgbArg<ini_args::kBaseColorArg, Blue>,
            White>>;
using IniBaseSequels =
    Layers<
        AudioFlicker<
            RgbArg<ini_args::kBaseColorArg, Blue>,
            RgbArg<ini_args::kAltColorArg, Rgb<180, 180, 255>>>,
        AlphaL<
            BrownNoiseFlickerL<
                RgbArg<ini_args::kAltColorArg, Rgb<180, 180, 255>>,
                IntArg<ini_args::kNoiseMixArg, 80>>,
            IntArg<ini_args::kFlickerDepthArg, 9000>>>;
using IniBaseAncient =
    StripesX<
        IntArg<ini_args::kStripeWidthArg, 5000>,
        IntArg<ini_args::kStripeSpeedArg, 250>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 9000>,
            Black,
            RgbArg<ini_args::kBaseColorArg, Green>>,
        Mix<
            IntArg<ini_args::kWarmShiftArg, 2000>,
            RgbArg<ini_args::kAltColorArg, Yellow>,
            RgbArg<ini_args::kArg3, Rgb<255, 180, 80>>>,
        Mix<
            IntArg<ini_args::kBaseContrastArg, 18000>,
            Black,
            RgbArg<ini_args::kArg3, Rgb<255, 180, 80>>>>;

template<class BASE>
StyleAllocator IniStyleAllocatorPtr() {
  return StylePtr<IniPrimaryBlade<BASE>>();
}

// --- Schema v2 templates ---
template<class BASE, class OUT_TR = TrWipe<300>, class IN_TR = TrWipeIn<500>, class OFF = Rgb<0, 0, 0>>
using IniResponsiveBladeV2 =
    InOutTr<
        Layers<
            BASE,
            ResponsiveLockupL<RgbArg<ini_args::kLockupColorArg, White>, TrInstant, TrFade<100>, Int<26000>>,
            ResponsiveLightningBlockL<RgbArg<ini_args::kLbColorArg, White>>,
            ResponsiveMeltL<Mix<TwistAngle<>, Red, Yellow>>,
            ResponsiveDragL<RgbArg<ini_args::kDragColorArg, White>>,
            ResponsiveClashL<RgbArg<ini_args::kClashColorArg, White>, TrInstant, TrFade<200>, Int<26000>>,
            ResponsiveBlastL<RgbArg<ini_args::kBlastColorArg, White>>,
            ResponsiveBlastWaveL<RgbArg<ini_args::kBlastColorArg, White>>,
            ResponsiveBlastFadeL<RgbArg<ini_args::kBlastColorArg, White>>,
            ResponsiveStabL<RgbArg<ini_args::kStabColorArg, White>>>,
        OUT_TR,
        IN_TR,
        OFF>;

using IniBaseStandardV2 = IniResponsiveBladeV2<RgbArg<ini_args::kBaseColorArg, Blue>>;

using IniBaseAudioFlickerV2 =
    IniResponsiveBladeV2<
        AudioFlicker<
            RgbArg<ini_args::kBaseColorArg, Blue>,
            RgbArg<ini_args::kAltColorArg, White>>>;

using IniBaseHumpFlickerV2 =
    IniResponsiveBladeV2<
        Layers<
            RgbArg<ini_args::kBaseColorArg, Magenta>,
            AlphaL<
                RgbArg<ini_args::kAltColorArg, White>,
                Mult<
                    HumpFlickerF<50>,
                    IntArg<ini_args::kStyleOptionArg, 32768>>>>>;

using IniBasePulsingStripesV2 =
    IniResponsiveBladeV2<
        StripesX<
            IntArg<ini_args::kStyleOptionArg, 3000>,
            IntArg<ini_args::kIgnitionOptionArg, -3000>,
            RgbArg<ini_args::kBaseColorArg, Blue>,
            Mix<Int<12000>, Black, RgbArg<ini_args::kBaseColorArg, Blue>>,
            PulsingX<
                RgbArg<ini_args::kBaseColorArg, Blue>,
                Mix<Int<8000>, Black, RgbArg<ini_args::kBaseColorArg, Blue>>,
                IntArg<ini_args::kSwingOptionArg, 1400>>>>;

using IniBaseEnergyV2 =
    IniResponsiveBladeV2<
        AudioFlicker<
            BrownNoiseFlicker<
                RgbArg<ini_args::kAltColorArg, White>,
                StripesX<
                    IntArg<ini_args::kStyleOptionArg, 5000>,
                    IntArg<ini_args::kIgnitionOptionArg, -300>,
                    Mix<Int<7710>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                    Mix<Int<25700>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                    Mix<Int<1285>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                    Mix<Int<16384>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>>,
                300>,
            Mix<Int<6425>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>, White>>>;

using IniBaseFireUnstableV2 =
    IniResponsiveBladeV2<
        StaticFire<
            BrownNoiseFlicker<
                RgbArg<ini_args::kBaseColorArg, Red>,
                RandomPerLEDFlicker<
                    Mix<Int<3213>, Black, RgbArg<ini_args::kAltColorArg, Orange>>,
                    Mix<Int<7710>, Black, RgbArg<ini_args::kAltColor2Arg, Orange>>>,
                300>,
            Mix<Int<10280>, Black, RgbArg<ini_args::kAltColor3Arg, Yellow>>,
            0,
            6,
            10,
            1000,
            2>>;

using IniBasePlasmaBladeV2 =
    IniResponsiveBladeV2<
        StaticFire<
            Mix<
                SmoothStep<Int<2000>, Int<-2000>>,
                StripesX<
                    IntArg<ini_args::kStyleOptionArg, 16000>,
                    IntArg<ini_args::kIgnitionOptionArg, -3900>,
                    RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>,
                    Mix<Int<8172>, RgbArg<ini_args::kAltColorArg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                    Mix<Int<16384>, RgbArg<ini_args::kAltColorArg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                    StripesX<
                        IntArg<ini_args::kStyleOption2Arg, 2500>,
                        IntArg<ini_args::kStyleOption3Arg, -3500>,
                        RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>,
                        RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>,
                        RgbArg<ini_args::kAltColor2Arg, Blue>,
                        Mix<Int<16000>, RgbArg<ini_args::kAltColorArg, Black>, RgbArg<ini_args::kBaseColorArg, Rgb<100, 100, 150>>>,
                        RgbArg<ini_args::kAltColor3Arg, DodgerBlue>>>,
                White>,
            Blue,
            0,
            6,
            1,
            2000,
            3>>;

using IniBaseRainbowBladeV2 = IniResponsiveBladeV2<Rainbow>;

using IniBaseEnergyBladeV2 =
    IniResponsiveBladeV2<
        Remap<
            CenterDistF<>,
            StripesX<
                IntArg<ini_args::kStyleOptionArg, 2000>,
                IntArg<ini_args::kIgnitionOptionArg, -3000>,
                AudioFlicker<RgbArg<ini_args::kAltColorArg, White>, RgbArg<ini_args::kBaseColorArg, Blue>>,
                Mix<Int<3855>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Blue>>,
                AudioFlicker<RgbArg<ini_args::kBaseColorArg, Blue>, RgbArg<ini_args::kAltColorArg, White>>,
                Rgb<50, 50, 75>>>>;

using IniBaseLavaBladeV2 =
    IniResponsiveBladeV2<
        StripesX<
            Sin<IntArg<ini_args::kStyleOptionArg, 4>, Int<3000>, Int<6000>>,
            Scale<TwistAngle<>, Int<-50>, Int<-100>>,
            PulsingX<
                Mix<Int<6425>, RgbArg<ini_args::kBaseColorArg, Blue>, RgbArg<ini_args::kAltColorArg, White>>,
                Mix<Int<2570>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Blue>>,
                IntArg<ini_args::kIgnitionOptionArg, 3000>>,
            Mix<Int<7710>, Black, RgbArg<ini_args::kBaseColorArg, Blue>>,
            PulsingX<
                Mix<Int<6425>, RgbArg<ini_args::kAltColor2Arg, Black>, RgbArg<ini_args::kBaseColorArg, Blue>>,
                Mix<Int<6425>, RgbArg<ini_args::kBaseColorArg, Blue>, RgbArg<ini_args::kAltColorArg, White>>,
                IntArg<ini_args::kSwingOptionArg, 2000>>>>;

using IniBaseSparkleBladeV2 =
    IniResponsiveBladeV2<Sparkle<RgbArg<ini_args::kBaseColorArg, Blue>, RgbArg<ini_args::kAltColorArg, White>>>;

using IniBaseFireBladeV2 =
    IniResponsiveBladeV2<
        StyleFire<RgbArg<ini_args::kBaseColorArg, Blue>, RgbArg<ini_args::kAltColorArg, Cyan>>,
        TrWipe<300>,
        TrFade<500>>;

using IniBasePulseAccentV2 =
    IniResponsiveBladeV2<
        PulsingX<
            RgbArg<ini_args::kBaseColorArg, Blue>,
            RgbArg<ini_args::kAltColorArg, Red>,
            IntArg<ini_args::kStyleOptionArg, 400>>,
        TrFade<300>,
        TrFade<500>,
        PulsingX<
            RgbArg<ini_args::kOffColorArg, Black>,
            Mix<Int<16384>, Black, RgbArg<ini_args::kOffColorArg, Black>>,
            IntArg<ini_args::kStyleOption2Arg, 1600>>>;

using IniBaseBlinkAccentV2 =
    IniResponsiveBladeV2<
        BlinkingX<
            RgbArg<ini_args::kBaseColorArg, Red>,
            RgbArg<ini_args::kAltColorArg, Blue>,
            IntArg<ini_args::kStyleOptionArg, 1000>,
            IntArg<ini_args::kIgnitionOptionArg, 500>>,
        TrFade<300>,
        TrFade<500>,
        BlinkingX<
            RgbArg<ini_args::kOffColorArg, Black>,
            Black,
            IntArg<ini_args::kStyleOption2Arg, 2000>,
            IntArg<ini_args::kStyleOption3Arg, 500>>>;

using IniBaseRandomBlinkAccentV2 =
    IniResponsiveBladeV2<
        RandomBlinkX<
            IntArg<ini_args::kStyleOptionArg, 6000>,
            RgbArg<ini_args::kBaseColorArg, White>,
            RgbArg<ini_args::kAltColorArg, Black>>,
        TrFade<300>,
        TrFade<500>,
        RandomBlinkX<
            IntArg<ini_args::kStyleOption2Arg, 3000>,
            RgbArg<ini_args::kOffColorArg, Black>,
            Black>>;

using IniBaseColorCycleAccentV2 =
    IniResponsiveBladeV2<
        ColorCycle<RgbArg<ini_args::kBaseColorArg, Blue>, 25, 100, RgbArg<ini_args::kAltColorArg, Cyan>, 100, 3000, 5000>,
        TrFade<300>,
        TrFade<300>,
        ColorCycle<RgbArg<ini_args::kOffColorArg, Black>, 25, 100, RgbArg<ini_args::kAltColorArg, Cyan>, 100, 3000, 5000>>;

template<class STYLE>
StyleAllocator IniDirectStyleAllocatorPtr() {
  return StylePtr<STYLE>();
}

#endif  // STYLES_INI_STYLE_TEMPLATES_H
