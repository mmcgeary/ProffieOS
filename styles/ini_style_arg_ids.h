#ifndef STYLES_INI_STYLE_ARG_IDS_H
#define STYLES_INI_STYLE_ARG_IDS_H

namespace ini_style_args {

enum ArgId : int {
  kBaseColorArg = 1,
  kAltColorArg = 2,
  kStyleOptionArg = 3,
  kIgnitionOptionArg = 4,
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
  kAltColor2Arg = 31,
  kAltColor3Arg = 32,
  kStyleOption2Arg = 33,
  kStyleOption3Arg = 34,
  kIgnitionOption2Arg = 35,
  kRetractionOption2Arg = 36,
  kRetractionOptionArg = 37,
  kSwingOptionArg = 38,
  kIgnitionDelayArg = 39,
  kRetractionDelayArg = 40,
  kLockupPositionArg = 41,
  kDragSizeArg = 42,
  kMeltSizeArg = 43,
  kSwingColorArg = 44,
  kEmitterSizeArg = 45,
  kPreonColorArg = 46,
  kPreonOptionArg = 47,
  kPreonSizeArg = 48,
  kRetractionColorArg = 49,
  kRetractionCoolDownArg = 50,
  kPostOffColorArg = 51,
  kIgnitionColorArg = 52,
  kIgnitionPowerUpArg = 53,
  kArgCount = kRainbowSpeedArg,
  kExtendedArgCount = kIgnitionPowerUpArg,
};

// Legacy aliases kept for compatibility with existing template code.
static constexpr int kArg3 = kStyleOptionArg;
static constexpr int kArg4 = kIgnitionOptionArg;

static constexpr int kFirstTuningArg = kFlickerDepthArg;
static constexpr int kTuningArgCount = kRainbowSpeedArg - kFlickerDepthArg + 1;

}  // namespace ini_style_args

#endif  // STYLES_INI_STYLE_ARG_IDS_H
