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
  kLockupFadeArg = 54,
  kClashFadeArg = 55,
  kLockupSizeArg = 56,
  kMeltBaseArg = 57,
  kMeltAltArg = 58,
  kPulseSpeedArg = 59,
  kHumpWidthArg = 60,
  kCycleOffRpmArg = 61,
  kCycleOnRpmArg = 62,
  kCycleOffPctArg = 63,
  kCycleOnPctArg = 64,
  kCycleFadeArg = 65,
  kArgCount = kRainbowSpeedArg,
  kExtendedArgCount = kCycleFadeArg,
};

// Legacy aliases kept for compatibility with existing template code.
static constexpr int kArg3 = kStyleOptionArg;
static constexpr int kArg4 = kIgnitionOptionArg;

static constexpr int kFirstTuningArg = kFlickerDepthArg;
static constexpr int kTuningArgCount = kRainbowSpeedArg - kFlickerDepthArg + 1;

}  // namespace ini_style_args

#define BASE_COLOR_ARG ini_style_args::kBaseColorArg
#define ALT_COLOR_ARG ini_style_args::kAltColorArg
#define ALT_COLOR2_ARG ini_style_args::kAltColor2Arg
#define ALT_COLOR3_ARG ini_style_args::kAltColor3Arg
#define STYLE_OPTION_ARG ini_style_args::kStyleOptionArg
#define IGNITION_OPTION_ARG ini_style_args::kIgnitionOptionArg
#define IGNITION_OPTION2_ARG ini_style_args::kIgnitionOption2Arg
#define RETRACTION_OPTION_ARG ini_style_args::kRetractionOptionArg
#define RETRACTION_OPTION2_ARG ini_style_args::kRetractionOption2Arg
#define SWING_OPTION_ARG ini_style_args::kSwingOptionArg
#define IGNITION_TIME_ARG ini_style_args::kIgnitionTimeArg
#define RETRACTION_TIME_ARG ini_style_args::kRetractionTimeArg
#define IGNITION_DELAY_ARG ini_style_args::kIgnitionDelayArg
#define RETRACTION_DELAY_ARG ini_style_args::kRetractionDelayArg
#define IGNITION_POWER_UP_ARG ini_style_args::kIgnitionPowerUpArg
#define IGNITION_COLOR_ARG ini_style_args::kIgnitionColorArg
#define RETRACTION_COLOR_ARG ini_style_args::kRetractionColorArg
#define RETRACTION_COOLDOWN_ARG ini_style_args::kRetractionCoolDownArg
#define BLAST_COLOR_ARG ini_style_args::kBlastColorArg
#define CLASH_COLOR_ARG ini_style_args::kClashColorArg
#define LOCKUP_COLOR_ARG ini_style_args::kLockupColorArg
#define LB_COLOR_ARG ini_style_args::kLbColorArg
#define DRAG_COLOR_ARG ini_style_args::kDragColorArg
#define STAB_COLOR_ARG ini_style_args::kStabColorArg
#define EMITTER_COLOR_ARG ini_style_args::kEmitterColorArg
#define PREON_COLOR_ARG ini_style_args::kPreonColorArg
#define PREON_OPTION_ARG ini_style_args::kPreonOptionArg
#define PREON_SIZE_ARG ini_style_args::kPreonSizeArg
#define OFF_COLOR_ARG ini_style_args::kOffColorArg
#define POST_OFF_COLOR_ARG ini_style_args::kPostOffColorArg
#define SWING_COLOR_ARG ini_style_args::kSwingColorArg
#define EMITTER_SIZE_ARG ini_style_args::kEmitterSizeArg
#define DRAG_SIZE_ARG ini_style_args::kDragSizeArg
#define MELT_SIZE_ARG ini_style_args::kMeltSizeArg
#define LOCKUP_POSITION_ARG ini_style_args::kLockupPositionArg
#define LOCKUP_FADE_ARG ini_style_args::kLockupFadeArg
#define CLASH_FADE_ARG ini_style_args::kClashFadeArg
#define LOCKUP_SIZE_ARG ini_style_args::kLockupSizeArg
#define MELT_BASE_ARG ini_style_args::kMeltBaseArg
#define MELT_ALT_ARG ini_style_args::kMeltAltArg
#define FLICKER_DEPTH_ARG ini_style_args::kFlickerDepthArg
#define FLICKER_SPEED_ARG ini_style_args::kFlickerSpeedArg
#define STRIPE_WIDTH_ARG ini_style_args::kStripeWidthArg
#define STRIPE_SPEED_ARG ini_style_args::kStripeSpeedArg
#define MOTION_GAIN_ARG ini_style_args::kMotionGainArg
#define NOISE_MIX_ARG ini_style_args::kNoiseMixArg
#define BASE_CONTRAST_ARG ini_style_args::kBaseContrastArg
#define DRIFT_RATE_ARG ini_style_args::kDriftRateArg
#define WARM_SHIFT_ARG ini_style_args::kWarmShiftArg
#define JITTER_AMOUNT_ARG ini_style_args::kJitterAmountArg
#define SPARK_MIX_ARG ini_style_args::kSparkMixArg
#define HEAT_RAND_ARG ini_style_args::kHeatRandArg
#define FIRE_COOLING_ARG ini_style_args::kFireCoolingArg
#define RAINBOW_SPEED_ARG ini_style_args::kRainbowSpeedArg
#define PULSE_SPEED_ARG ini_style_args::kPulseSpeedArg
#define HUMP_WIDTH_ARG ini_style_args::kHumpWidthArg
#define CYCLE_OFF_RPM_ARG ini_style_args::kCycleOffRpmArg
#define CYCLE_ON_RPM_ARG ini_style_args::kCycleOnRpmArg
#define CYCLE_OFF_PCT_ARG ini_style_args::kCycleOffPctArg
#define CYCLE_ON_PCT_ARG ini_style_args::kCycleOnPctArg
#define CYCLE_FADE_ARG ini_style_args::kCycleFadeArg

#endif  // STYLES_INI_STYLE_ARG_IDS_H
