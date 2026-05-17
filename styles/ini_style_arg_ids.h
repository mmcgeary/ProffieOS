#ifndef STYLES_INI_STYLE_ARG_IDS_H
#define STYLES_INI_STYLE_ARG_IDS_H

namespace ini_style_args {

enum ArgId : int {
  kBaseColorArg = 1,
  kAltColorArg = 2,
  kArg3 = 3,
  kArg4 = 4,
  kBlastColorArg = 5,
  kClashColorArg = 6,
  kLockupColorArg = 7,
  kLbColorArg = 8,
  kDragColorArg = 9,
  kStabColorArg = 10,
  kEmitterColorArg = 11,
  kIgnitionTimeArg = 12,
  kRetractionTimeArg = 13,
  kOffColorArg = 14,
  kOffModeArg = 15,
  kOffRateMsArg = 16,
  kFlickerDepthArg = 17,
  kFlickerSpeedArg = 18,
  kStripeWidthArg = 19,
  kStripeSpeedArg = 20,
  kMotionGainArg = 21,
  kNoiseMixArg = 22,
  kBaseContrastArg = 23,
  kDriftRateArg = 24,
  kWarmShiftArg = 25,
  kJitterAmountArg = 26,
  kSparkMixArg = 27,
  kHeatRandArg = 28,
  kFireCoolingArg = 29,
  kRainbowSpeedArg = 30,
  kArgCount = kRainbowSpeedArg,
};

static constexpr int kFirstTuningArg = kFlickerDepthArg;
static constexpr int kTuningArgCount = kRainbowSpeedArg - kFlickerDepthArg + 1;

}  // namespace ini_style_args

#endif  // STYLES_INI_STYLE_ARG_IDS_H
