#ifndef PROPS_INI_TUNING_ARG_TABLE_H
#define PROPS_INI_TUNING_ARG_TABLE_H

#include "../styles/ini_style_arg_ids.h"

#define INI_TUNING_ARG_TABLE(X) \
  X(flicker_depth, kFlickerDepthArg, "flicker_depth", 12000) \
  X(flicker_speed, kFlickerSpeedArg, "flicker_speed", 1000) \
  X(stripe_width, kStripeWidthArg, "stripe_width", 5000) \
  X(stripe_speed, kStripeSpeedArg, "stripe_speed", 900) \
  X(motion_gain, kMotionGainArg, "motion_gain", 4096) \
  X(noise_mix, kNoiseMixArg, "noise_mix", 8000) \
  X(base_contrast, kBaseContrastArg, "base_contrast", 32768) \
  X(drift_rate, kDriftRateArg, "drift_rate", 600) \
  X(warm_shift, kWarmShiftArg, "warm_shift", 2000) \
  X(jitter_amount, kJitterAmountArg, "jitter_amount", 50) \
  X(spark_mix, kSparkMixArg, "spark_mix", 5000) \
  X(heat_rand, kHeatRandArg, "heat_rand", 4500) \
  X(fire_cooling, kFireCoolingArg, "fire_cooling", 55) \
  X(rainbow_speed, kRainbowSpeedArg, "rainbow_speed", 800)

#endif  // PROPS_INI_TUNING_ARG_TABLE_H
