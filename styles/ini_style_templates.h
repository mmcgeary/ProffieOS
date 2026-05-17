#ifndef STYLES_INI_STYLE_TEMPLATES_H
#define STYLES_INI_STYLE_TEMPLATES_H

#include "ini_style_arg_ids.h"
#include "../functions/int_arg.h"
#include "../functions/mult.h"
#include "../functions/scale.h"
#include "../transitions/fade.h"
#include "../transitions/wipe.h"
#include "audio_flicker.h"
#include "brown_noise_flicker.h"
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
#include "responsive_styles.h"
#include "rgb_arg.h"
#include "rotate_color.h"
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
                PulsingF<IntArg<ini_args::kFlickerSpeedArg, 1000>>,
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
                PulsingF<IntArg<ini_args::kArg3, 1400>>,
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

#endif  // STYLES_INI_STYLE_TEMPLATES_H
