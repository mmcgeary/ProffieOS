#ifndef STYLES_INI_STYLE_ARG_IDS_H
#define STYLES_INI_STYLE_ARG_IDS_H

namespace ini_style_args {

enum ArgId : int {
  kBaseColorArg = 1,
  kAltColorArg = 2,
  kStyleOptionArg = 3,
  kIgnitionOptionArg = 4,
  kIgnitionTimeArg = 5,
  kIgnitionDelayArg = 6,
  kIgnitionColorArg = 7,
  kIgnitionPowerUpArg = 8,
  kBlastColorArg = 9,
  kClashColorArg = 10,
  kLockupColorArg = 11,
  kLockupPositionArg = 12,
  kDragColorArg = 13,
  kDragSizeArg = 14,
  kLbColorArg = 15,
  kStabColorArg = 16,
  kMeltSizeArg = 17,
  kSwingColorArg = 18,
  kSwingOptionArg = 19,
  kEmitterColorArg = 20,
  kEmitterSizeArg = 21,
  kPreonColorArg = 22,
  kPreonOptionArg = 23,
  kPreonSizeArg = 24,
  kRetractionOptionArg = 25,
  kRetractionTimeArg = 26,
  kRetractionDelayArg = 27,
  kRetractionColorArg = 28,
  kRetractionCoolDownArg = 29,
  kPostOffColorArg = 30,
  kOffColorArg = 31,
  kOffOptionArg = 32, // Wait, edit_mode.h has OFF_OPTION_ARG = 32
  kAltColor2Arg = 33,
  kAltColor3Arg = 34,
  kStyleOption2Arg = 35,
  kStyleOption3Arg = 36,
  kIgnitionOption2Arg = 37,
  kRetractionOption2Arg = 38,

  // Custom ProffieOS-Companion tuning arguments start here!
  kFlickerDepthArg = 39,
  kFlickerSpeedArg = 40,
  kStripeWidthArg = 41,
  kStripeSpeedArg = 42,
  kMotionGainArg = 43,
  kNoiseMixArg = 44,
  kBaseContrastArg = 45,
  kDriftRateArg = 46,
  kWarmShiftArg = 47,
  kJitterAmountArg = 48,
  kSparkMixArg = 49,
  kHeatRandArg = 50,
  kFireCoolingArg = 51,
  kRainbowSpeedArg = 52,
  kLockupFadeArg = 53,
  kClashFadeArg = 54,
  kLockupSizeArg = 55,
  kMeltBaseArg = 56,
  kMeltAltArg = 57,
  kPulseSpeedArg = 58,
  kHumpWidthArg = 59,
  kCycleOffRpmArg = 60,
  kCycleOnRpmArg = 61,
  kCycleOffPctArg = 62,
  kCycleOnPctArg = 63,
  kCycleFadeArg = 64,
  kClashModeArg = 65,
  kBlastModeArg = 66,
  kLockupModeArg = 67,
  kIgnitionModeArg = 68,
  kRetractionModeArg = 69,
  kClashWidthArg = 70,
  kBlastSizeArg = 71,
  kBlastSpeedArg = 72,
  kSparkColorArg = 73,
  kSparkSizeArg = 74,

  kArgCount = kRainbowSpeedArg,
  kExtendedArgCount = kSparkSizeArg,
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
#define OFF_OPTION_ARG ini_style_args::kOffOptionArg
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
#define CLASH_MODE_ARG ini_style_args::kClashModeArg
#define BLAST_MODE_ARG ini_style_args::kBlastModeArg
#define LOCKUP_MODE_ARG ini_style_args::kLockupModeArg
#define IGNITION_MODE_ARG ini_style_args::kIgnitionModeArg
#define RETRACTION_MODE_ARG ini_style_args::kRetractionModeArg
#define CLASH_WIDTH_ARG ini_style_args::kClashWidthArg
#define BLAST_SIZE_ARG ini_style_args::kBlastSizeArg
#define BLAST_SPEED_ARG ini_style_args::kBlastSpeedArg
#define SPARK_COLOR_ARG ini_style_args::kSparkColorArg
#define SPARK_SIZE_ARG ini_style_args::kSparkSizeArg

#endif  // STYLES_INI_STYLE_ARG_IDS_H
