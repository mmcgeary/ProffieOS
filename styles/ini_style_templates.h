#ifndef STYLES_INI_STYLE_TEMPLATES_H
#define STYLES_INI_STYLE_TEMPLATES_H

#include "../functions/int_arg.h"
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
#include "random_per_led_flicker.h"
#include "responsive_styles.h"
#include "rgb_arg.h"
#include "rotate_color.h"
#include "strobe.h"
#include "stripes.h"
#include "style_ptr.h"

template<class BASE>
using IniPrimaryBlade = InOutTr<
    Layers<
        BASE,
        ResponsiveBlastL<RgbArg<5, White>>,
        ResponsiveClashL<RgbArg<6, White>>,
        LockupTrL<
            AlphaL<
                AudioFlicker<RgbArg<7, White>, Mix<Int<12288>, Black, RgbArg<7, White>>>,
                Bump<Int<16000>, Int<12000>>>,
            TrInstant,
            TrFade<300>,
            SaberBase::LOCKUP_NORMAL>,
        ResponsiveLightningBlockL<
            Strobe<RgbArg<8, White>, AudioFlicker<RgbArg<8, White>, Blue>, 50, 1>>,
        LockupTrL<
            AlphaL<RandomPerLEDFlickerL<RgbArg<9, White>>, SmoothStep<Int<28000>, Int<3000>>>,
            TrWipeIn<180>,
            TrFade<220>,
            SaberBase::LOCKUP_DRAG>,
        LockupTrL<
            AlphaL<
                HumpFlicker<
                    RgbArg<10, Rgb<255, 68, 0>>,
                    RotateColorsX<Int<3000>, RgbArg<10, Rgb<255, 68, 0>>>,
                    90>,
                SmoothStep<Int<28000>, Int<3200>>>,
            TrWipeIn<120>,
            TrWipe<200>,
            SaberBase::LOCKUP_MELT>>,
    TrWipeSparkTipX<RgbArg<11, White>, IntArg<12, 300>>,
    TrWipeInSparkTipX<RgbArg<11, White>, IntArg<13, 800>>,
    ColorSelect<
        IntArg<15, 1>,
        TrInstant,
        RgbArg<14, Black>,
        PulsingX<RgbArg<14, Black>, Mix<Int<16384>, Black, RgbArg<14, Black>>, IntArg<16, 2200>>,
        RandomBlinkX<IntArg<16, 2200>, RgbArg<14, Black>, Black>>>;

using IniBaseStandard = RgbArg<1, CYAN>;
using IniBaseHumpFlicker = HumpFlicker<RgbArg<1, CYAN>, RgbArg<2, WHITE>, 100>;
using IniBaseUnstable =
    BrownNoiseFlicker<
        Strobe<RgbArg<1, Rgb<150, 0, 0>>, RgbArg<2, Red>, 50, 5>,
        Strobe<RgbArg<3, Rgb<255, 40, 0>>, Black, 50, 1>,
        120>;
using IniBaseFire = StaticFire<RgbArg<1, RED>, RgbArg<2, YELLOW>, 0, 2, 0, 1800, 6>;
using IniBaseRainbow = Rainbow;
using IniBaseStrobe = StrobeX<RgbArg<1, BLACK>, RgbArg<2, WHITE>, IntArg<3, 15>, IntArg<4, 1>>;
using IniBasePulse = PulsingX<RgbArg<1, CYAN>, RgbArg<2, WHITE>, IntArg<3, 1400>>;
using IniBaseRotoscope =
    HumpFlicker<
        RgbArg<1, Rgb<200, 200, 255>>,
        Stripes<
            35000,
            -200,
            Mix<Int<14000>, Black, RgbArg<1, Rgb<200, 200, 255>>>,
            Mix<Int<24000>, Black, RgbArg<1, Rgb<200, 200, 255>>>,
            Mix<Int<28000>, Black, RgbArg<1, Rgb<200, 200, 255>>>>,
        100>;
using IniBaseGhostly =
    AudioFlicker<
        Mix<Int<20000>, Black, RgbArg<1, Rgb<200, 255, 255>>>,
        Mix<Int<12000>, Black, RgbArg<2, Rgb<240, 255, 255>>>>;
using IniBaseLightning =
    StrobeX<
        BrownNoiseFlicker<RgbArg<1, Blue>, RgbArg<2, White>, 120>,
        RgbArg<3, White>,
        IntArg<4, 25>,
        Int<2>>;
using IniBaseDarksaber =
    Stripes<
        2800,
        -11000,
        Mix<Int<26000>, Black, RgbArg<1, White>>,
        RgbArg<2, White>,
        Mix<Int<8000>, Black, RgbArg<1, White>>>;
using IniBaseKylo =
    HumpFlicker<
        BrownNoiseFlicker<RgbArg<1, Rgb<170, 0, 0>>, RgbArg<2, Red>, 150>,
        RgbArg<3, Rgb<255, 180, 0>>,
        90>;
using IniBasePrequels = AudioFlicker<RgbArg<1, Blue>, Mix<Int<4096>, RgbArg<1, Blue>, White>>;
using IniBaseSequels =
    BrownNoiseFlicker<
        AudioFlicker<RgbArg<1, Blue>, RgbArg<2, Rgb<180, 180, 255>>>,
        RgbArg<2, Rgb<180, 180, 255>>,
        80>;
using IniBaseAncient =
    Stripes<
        5000,
        -250,
        Mix<Int<9000>, Black, RgbArg<1, Green>>,
        RgbArg<2, Yellow>,
        Mix<Int<18000>, Black, RgbArg<3, Rgb<255, 180, 80>>>>;

template<class BASE>
StyleAllocator IniStyleAllocatorPtr() {
  return StylePtr<IniPrimaryBlade<BASE>>();
}

#endif  // STYLES_INI_STYLE_TEMPLATES_H
