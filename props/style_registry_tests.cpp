#define PROFFIE_TEST

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#define HEX 16
#define SCOPED_PROFILER() do {} while (0)
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

uint32_t micros() { return 0; }
uint32_t millis() { return 0; }
int random(int x) { return x > 0 ? (rand() % x) : 0; }

int32_t clampi32(int32_t x, int32_t a, int32_t b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

float fract(float x) { return x - floorf(x); }

char* itoa(int value, char* str, int radix) {
  if (radix != 10) {
    str[0] = 0;
    return str;
  }
  snprintf(str, 16, "%d", value);
  return str;
}

#include "../styles/ini_style_arg_ids.h"
#include "blade_bank_utils.h"
#define private public
#include "ini_loader.h"
#undef private
#include "style_registry.h"

struct SaberBase {
  enum LockupType {
    LOCKUP_NONE = 0,
    LOCKUP_NORMAL,
    LOCKUP_DRAG,
    LOCKUP_MELT,
    LOCKUP_LIGHTNING_BLOCK,
  };
  static LockupType Lockup() { return LOCKUP_NONE; }
  static void SetLockup(LockupType) {}
  static void DoBeginLockup() {}
  static void DoBlast() {}
  static void DoClash() {}
  static void DoForce() {}
  static void DoStab() {}
};

#include "action_dispatch.h"

static std::vector<std::string> SplitTokens(const char* input) {
  std::vector<std::string> out;
  const char* p = input;
  while (*p) {
    while (*p == ' ') {
      ++p;
    }
    if (!*p) break;
    const char* start = p;
    while (*p && *p != ' ') {
      ++p;
    }
    out.emplace_back(start, static_cast<size_t>(p - start));
  }
  return out;
}

#define CHECK(X) do { \
  if (!(X)) { \
    fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__, #X); \
    exit(1); \
  } \
} while (0)

static void CheckTokenEq(const std::vector<std::string>& tokens, size_t idx, const char* expected) {
  CHECK(idx < tokens.size());
  CHECK(tokens[idx] == expected);
}

static std::string AsString(unsigned int value) {
  return std::to_string(value);
}

static void TestArgIndexConstants() {
  static_assert(ini_style_args::kBaseColorArg == 1, "base color arg index changed");
  static_assert(ini_style_args::kAltColorArg == 2, "alt color arg index changed");
  static_assert(ini_style_args::kArgCount == 30, "arg count changed");
  static_assert(
      ini_style_args::kFirstTuningArg == ini_style_args::kFlickerDepthArg,
      "first tuning arg mismatch");
  static_assert(
      ini_style_args::kTuningArgCount ==
          (ini_style_args::kRainbowSpeedArg - ini_style_args::kFlickerDepthArg + 1),
      "tuning arg count mismatch");
}

static void InitPresetForTokenTests(IniPreset* p) {
  strcpy(p->base_color, "1,2,3");
  strcpy(p->alt_color, "4,5,6");
  strcpy(p->blast_color, "7,8,9");
  strcpy(p->clash_color, "10,11,12");
  strcpy(p->lockup_color, "13,14,15");
  strcpy(p->lb_color, "16,17,18");
  strcpy(p->drag_color, "19,20,21");
  strcpy(p->stab_color, "22,23,24");
  strcpy(p->emitter_color, "25,26,27");
  strcpy(p->swing_color, "28,29,30");
  strcpy(p->off_color, "31,32,33");

  p->ignition_time = 345;
  p->retraction_time = 678;
  p->off_mode = OFF_MODE_RANDOM;
  p->off_rate_ms = 999;

  p->flicker_depth = 101;
  p->flicker_speed = 102;
  p->stripe_width = 103;
  p->stripe_speed = 104;
  p->motion_gain = 105;
  p->noise_mix = 106;
  p->base_contrast = 107;
  p->drift_rate = 108;
  p->warm_shift = 109;
  p->jitter_amount = 110;
  p->spark_mix = 111;
  p->heat_rand = 112;
  p->fire_cooling = 113;
  p->rainbow_speed = 114;
}

static void CheckSharedArgs(const std::vector<std::string>& tokens, const IniPreset& p) {
  CheckTokenEq(tokens, ini_style_args::kBlastColorArg, p.blast_color);
  CheckTokenEq(tokens, ini_style_args::kClashColorArg, p.clash_color);
  CheckTokenEq(tokens, ini_style_args::kLockupColorArg, p.lockup_color);
  CheckTokenEq(tokens, ini_style_args::kLbColorArg, p.lb_color);
  CheckTokenEq(tokens, ini_style_args::kDragColorArg, p.drag_color);
  CheckTokenEq(tokens, ini_style_args::kStabColorArg, p.stab_color);
  CheckTokenEq(tokens, ini_style_args::kEmitterColorArg, p.emitter_color);
  CheckTokenEq(tokens, ini_style_args::kIgnitionTimeArg, AsString(p.ignition_time).c_str());
  CheckTokenEq(tokens, ini_style_args::kRetractionTimeArg, AsString(p.retraction_time).c_str());
  CheckTokenEq(tokens, ini_style_args::kOffColorArg, p.off_color);
  CheckTokenEq(tokens, ini_style_args::kOffModeArg, "2");
  CheckTokenEq(tokens, ini_style_args::kOffRateMsArg, AsString(p.off_rate_ms).c_str());

  CheckTokenEq(tokens, ini_style_args::kFlickerDepthArg, AsString(p.flicker_depth).c_str());
  CheckTokenEq(tokens, ini_style_args::kFlickerSpeedArg, AsString(p.flicker_speed).c_str());
  CheckTokenEq(tokens, ini_style_args::kStripeWidthArg, AsString(p.stripe_width).c_str());
  CheckTokenEq(tokens, ini_style_args::kStripeSpeedArg, AsString(p.stripe_speed).c_str());
  CheckTokenEq(tokens, ini_style_args::kMotionGainArg, AsString(p.motion_gain).c_str());
  CheckTokenEq(tokens, ini_style_args::kNoiseMixArg, AsString(p.noise_mix).c_str());
  CheckTokenEq(tokens, ini_style_args::kBaseContrastArg, AsString(p.base_contrast).c_str());
  CheckTokenEq(tokens, ini_style_args::kDriftRateArg, AsString(p.drift_rate).c_str());
  CheckTokenEq(tokens, ini_style_args::kWarmShiftArg, AsString(p.warm_shift).c_str());
  CheckTokenEq(tokens, ini_style_args::kJitterAmountArg, AsString(p.jitter_amount).c_str());
  CheckTokenEq(tokens, ini_style_args::kSparkMixArg, AsString(p.spark_mix).c_str());
  CheckTokenEq(tokens, ini_style_args::kHeatRandArg, AsString(p.heat_rand).c_str());
  CheckTokenEq(tokens, ini_style_args::kFireCoolingArg, AsString(p.fire_cooling).c_str());
  CheckTokenEq(tokens, ini_style_args::kRainbowSpeedArg, AsString(p.rainbow_speed).c_str());
}

static void TestStandardIncludesAllTuningArgs() {
  IniPreset p;
  p.SetDefaults();
  InitPresetForTokenTests(&p);
  char buf[1024];
  CHECK(BuildStandard(&p, buf, sizeof(buf)) > 0);
  const auto tokens = SplitTokens(buf);

  CHECK(tokens.size() == static_cast<size_t>(ini_style_args::kArgCount + 1));
  CheckTokenEq(tokens, 0, "ini_standard");
  CheckSharedArgs(tokens, p);
}

static void TestNumericArgPositionsRemainStable() {
  IniPreset p;
  p.SetDefaults();
  InitPresetForTokenTests(&p);
  p.strobe_freq = 27;
  p.strobe_ms = 6;

  char buf[1024];
  CHECK(BuildStrobe(&p, buf, sizeof(buf)) > 0);
  const auto tokens = SplitTokens(buf);

  CHECK(tokens.size() == static_cast<size_t>(ini_style_args::kArgCount + 1));
  CheckTokenEq(tokens, 0, "ini_strobe");
  CheckTokenEq(tokens, ini_style_args::kArg3, "27");
  CheckTokenEq(tokens, ini_style_args::kArg4, "6");
  CheckSharedArgs(tokens, p);
}

static void CheckArg1234ByStyleName(const char* style_name,
                                    const std::vector<std::string>& tokens,
                                    const IniPreset& p) {
  CheckTokenEq(tokens, ini_style_args::kBaseColorArg, p.base_color);
  CheckTokenEq(tokens, ini_style_args::kAltColorArg, p.alt_color);

  if (!strcmp(style_name, "strobe")) {
    CheckTokenEq(tokens, ini_style_args::kArg3, AsString(p.strobe_freq).c_str());
    CheckTokenEq(tokens, ini_style_args::kArg4, AsString(p.strobe_ms).c_str());
    return;
  }
  if (!strcmp(style_name, "pulse")) {
    CheckTokenEq(tokens, ini_style_args::kArg3, AsString(p.pulse_rate).c_str());
    CheckTokenEq(tokens, ini_style_args::kArg4, AsString(p.pulse_depth).c_str());
    return;
  }
  if (!strcmp(style_name, "lightning")) {
    CheckTokenEq(tokens, ini_style_args::kArg3, p.clash_color);
    CheckTokenEq(tokens, ini_style_args::kArg4, AsString(p.strobe_freq).c_str());
    return;
  }
  if (!strcmp(style_name, "unstable") || !strcmp(style_name, "kylo")) {
    CheckTokenEq(tokens, ini_style_args::kArg3, p.blast_color);
    CheckTokenEq(tokens, ini_style_args::kArg4, p.clash_color);
    return;
  }

  CheckTokenEq(tokens, ini_style_args::kArg3, p.swing_color);
  CheckTokenEq(tokens, ini_style_args::kArg4, p.clash_color);
}

static void TestEveryMainStyleBuildContract() {
  IniPreset p;
  p.SetDefaults();
  InitPresetForTokenTests(&p);
  p.strobe_freq = 29;
  p.strobe_ms = 7;
  p.pulse_rate = 1333;
  p.pulse_depth = 22222;

  for (int i = 0; i < INI_STYLE_COUNT; i++) {
    const IniStyleEntry& entry = ini_style_registry[i];
    char buf[1024];
    CHECK(entry.build(&p, buf, sizeof(buf)) > 0);
    const auto tokens = SplitTokens(buf);
    CHECK(tokens.size() == static_cast<size_t>(ini_style_args::kArgCount + 1));

    const std::string expected_parser = std::string("ini_") + entry.name;
    CheckTokenEq(tokens, 0, expected_parser.c_str());
    CheckArg1234ByStyleName(entry.name, tokens, p);
    CheckSharedArgs(tokens, p);
  }
}

static void TestBaseContrastAliasAndClamps() {
  IniPreset p;
  p.SetDefaults();

  IniLoader::ParsePreset("core_contrast", "12345", &p);
  CHECK(p.base_contrast == 12345);
  IniLoader::ParsePreset("base_contrast", "23456", &p);
  CHECK(p.base_contrast == 23456);

  IniLoader::ParsePreset("flicker_depth", "999999", &p);
  IniLoader::ParsePreset("flicker_speed", "-1", &p);
  IniLoader::ParsePreset("stripe_width", "-5", &p);
  IniLoader::ParsePreset("stripe_speed", "999999", &p);
  IniLoader::ParsePreset("motion_gain", "999999", &p);
  IniLoader::ParsePreset("noise_mix", "-1", &p);
  IniLoader::ParsePreset("base_contrast", "999999", &p);
  IniLoader::ParsePreset("pulse_rate", "-1", &p);
  IniLoader::ParsePreset("pulse_depth", "999999", &p);
  IniLoader::ParsePreset("strobe_freq", "999999", &p);
  IniLoader::ParsePreset("strobe_ms", "-1", &p);
  IniLoader::ParsePreset("drift_rate", "999999", &p);
  IniLoader::ParsePreset("warm_shift", "999999", &p);
  IniLoader::ParsePreset("jitter_amount", "-1", &p);
  IniLoader::ParsePreset("spark_mix", "999999", &p);
  IniLoader::ParsePreset("heat_rand", "-1", &p);
  IniLoader::ParsePreset("fire_cooling", "999999", &p);
  IniLoader::ParsePreset("rainbow_speed", "-1", &p);
  IniLoader::ParsePreset("off_rate_ms", "999999", &p);

  CHECK(p.flicker_depth == 32768);
  CHECK(p.flicker_speed == 1);
  CHECK(p.stripe_width == 1);
  CHECK(p.stripe_speed == 20000);
  CHECK(p.motion_gain == 32768);
  CHECK(p.noise_mix == 0);
  CHECK(p.base_contrast == 32768);
  CHECK(p.pulse_rate == 1);
  CHECK(p.pulse_depth == 32768);
  CHECK(p.strobe_freq == 200);
  CHECK(p.strobe_ms == 1);
  CHECK(p.drift_rate == 32768);
  CHECK(p.warm_shift == 32768);
  CHECK(p.jitter_amount == 1);
  CHECK(p.spark_mix == 32768);
  CHECK(p.heat_rand == 0);
  CHECK(p.fire_cooling == 255);
  CHECK(p.rainbow_speed == 1);
  CHECK(p.off_rate_ms == 60000);
}

static void TestStyleStringTruncationGuard() {
  IniPreset p;
  p.SetDefaults();
  InitPresetForTokenTests(&p);
  p.off_rate_ms = 54321;

  char full[1024];
  const int full_len = BuildStandard(&p, full, sizeof(full));
  CHECK(full_len > 0);

  std::vector<char> exact_fit(static_cast<size_t>(full_len));
  CHECK(BuildStandard(&p, exact_fit.data(), static_cast<int>(exact_fit.size())) < 0);

  std::vector<char> plus_null(static_cast<size_t>(full_len + 1));
  CHECK(BuildStandard(&p, plus_null.data(), static_cast<int>(plus_null.size())) == full_len);
}

static void TestBladeBankSelectionRules() {
  CHECK(ShouldUseBladeOutConfig(false, true));
  CHECK(!ShouldUseBladeOutConfig(true, true));
  CHECK(!ShouldUseBladeOutConfig(false, false));
  CHECK(!ShouldUseBladeOutConfig(true, false));
}

static void TestCopyGlobalAndActionsPreservesSourceValues() {
  RuntimeConfig src;
  src.SetDefaults();
  src.global.volume = 17;
  src.global.clash_threshold = 11;
  src.global.gesture_flags = 0x55;
  src.action_map_on[0] = ACTION_OFF;
  src.action_map_on[1] = ACTION_FORCE;
  src.action_map_off[0] = ACTION_ON;
  src.action_map_off[1] = ACTION_NEXT_PRESET;

  RuntimeConfig dst;
  dst.SetDefaults();
  dst.global.volume = 99;
  dst.action_map_on[0] = ACTION_NONE;
  dst.action_map_off[0] = ACTION_NONE;

  CopyGlobalAndActions(src, &dst);

  CHECK(dst.global.volume == src.global.volume);
  CHECK(dst.global.clash_threshold == src.global.clash_threshold);
  CHECK(dst.global.gesture_flags == src.global.gesture_flags);
  CHECK(dst.action_map_on[0] == src.action_map_on[0]);
  CHECK(dst.action_map_on[1] == src.action_map_on[1]);
  CHECK(dst.action_map_off[0] == src.action_map_off[0]);
  CHECK(dst.action_map_off[1] == src.action_map_off[1]);
}

static void TestApplyButtonProfileDefaultsPopulatesPowerSlots() {
  RuntimeConfig cfg;
  cfg.SetDefaults();
  CHECK(cfg.action_map_on[0] == ACTION_NONE);
  CHECK(cfg.action_map_off[0] == ACTION_NONE);

  ApplyButtonProfileDefaults(&cfg);

  CHECK(cfg.action_map_on[0] == ACTION_NONE);
  CHECK(cfg.action_map_off[0] == ACTION_ON_OR_VOLUME_UP);
}

static void TestIniLoadSequenceReappliesButtonDefaults() {
  RuntimeConfig cfg;
  cfg.SetDefaults();
  ApplyButtonProfileDefaults(&cfg);
  CHECK(cfg.action_map_on[0] == ACTION_NONE);
  CHECK(cfg.action_map_off[0] == ACTION_ON_OR_VOLUME_UP);

  // IniLoader::Load starts by resetting config defaults before parsing.
  cfg.SetDefaults();
  IniLoader::ParseGlobal("num_buttons", "1", &cfg.global);
  IniLoader::ParseGlobal("button_profile", "default", &cfg.global);
  IniLoader::FinalizeButtonMappings(&cfg);

  CHECK(cfg.action_map_on[0] == ACTION_BLAST);
  CHECK(cfg.action_map_off[0] == ACTION_ON_OR_VOLUME_UP);
}

static void TestBuildSaveDirPathForPresetsFile() {
  char path[128];
  BuildSaveDirPath("", "presets.ini", path, sizeof(path));
  CHECK(strcmp(path, "presets.ini") == 0);

  BuildSaveDirPath("blade", "presets.ini", path, sizeof(path));
  CHECK(strcmp(path, "blade/presets.ini") == 0);

  BuildSaveDirPath("noblade", "presets.ini", path, sizeof(path));
  CHECK(strcmp(path, "noblade/presets.ini") == 0);
}

static void TestSa22cProfileDefaultsOneButton() {
  RuntimeConfig cfg;
  cfg.SetDefaults();
  cfg.global.num_buttons = 1;
  ApplyButtonProfileDefaults(&cfg);

  CHECK(cfg.action_map_off[SLOT_PWR_CLICK] == ACTION_ON_OR_VOLUME_UP);
  CHECK(cfg.action_map_off[SLOT_PWR_LONG_CLICK] == ACTION_NEXT_PRESET_OR_VOLUME_DOWN);
  CHECK(cfg.action_map_off[SLOT_PWR_DOUBLE_CLICK] == ACTION_TRACK_PLAYER);
  CHECK(cfg.action_map_off[SLOT_PWR_DOUBLE_HOLD] == ACTION_ACTIVATE_MUTED);
  CHECK(cfg.action_map_off[SLOT_PWR_TRIPLE_CLICK] == ACTION_BATTERY_LEVEL);
  CHECK(cfg.action_map_off[SLOT_PWR_MOD_CLASH] == ACTION_TOGGLE_VOLUME_MENU);

  CHECK(cfg.action_map_on[SLOT_PWR_CLICK] == ACTION_BLAST);
  CHECK(cfg.action_map_on[SLOT_PWR_HOLD_LONG] == ACTION_OFF);
  CHECK(cfg.action_map_on[SLOT_PWR_DOUBLE_CLICK] == ACTION_BLAST);
  CHECK(cfg.action_map_on[SLOT_PWR_DOUBLE_HOLD] == ACTION_LIGHTNING_BLOCK);
  CHECK(cfg.action_map_on[SLOT_PWR_TRIPLE_HOLD] == ACTION_TOGGLE_BATTLE_MODE);
  CHECK(cfg.action_map_on[SLOT_PWR_MOD_CLASH] == ACTION_LOCKUP_OR_DRAG);
  CHECK(cfg.action_map_on[SLOT_PWR_MOD_STAB] == ACTION_MELT);
  CHECK(cfg.action_map_on[SLOT_PWR_MOD_SWING] == ACTION_TOGGLE_MULTI_BLAST);
  CHECK(cfg.action_map_on[SLOT_PWR_MOD_TWIST] == ACTION_FORCE_OR_COLOR_CHANGE);
}

static void TestSa22cProfileDefaultsTwoButton() {
  RuntimeConfig cfg;
  cfg.SetDefaults();
  cfg.global.num_buttons = 2;
  ApplyButtonProfileDefaults(&cfg);

  CHECK(cfg.action_map_off[SLOT_PWR_CLICK] == ACTION_ON_OR_VOLUME_UP);
  CHECK(cfg.action_map_off[SLOT_PWR_LONG_CLICK] == ACTION_TRACK_PLAYER);
  CHECK(cfg.action_map_off[SLOT_PWR_HOLD_LONG] == ACTION_PREV_PRESET_IF_NOT_VOLUME_MENU);
  CHECK(cfg.action_map_off[SLOT_PWR_DOUBLE_HOLD] == ACTION_ACTIVATE_MUTED);
  CHECK(cfg.action_map_off[SLOT_AUX_CLICK] == ACTION_NEXT_PRESET_OR_VOLUME_DOWN);
  CHECK(cfg.action_map_off[SLOT_AUX_LONG_CLICK] == ACTION_TOGGLE_VOLUME_MENU);
  CHECK(cfg.action_map_off[SLOT_AUX_HOLD_LONG] == ACTION_BATTERY_LEVEL);

  CHECK(cfg.action_map_on[SLOT_PWR_HOLD_MEDIUM] == ACTION_OFF);
  CHECK(cfg.action_map_on[SLOT_PWR_DOUBLE_CLICK] == ACTION_FORCE);
  CHECK(cfg.action_map_on[SLOT_PWR_DOUBLE_HOLD] == ACTION_LIGHTNING_BLOCK);
  CHECK(cfg.action_map_on[SLOT_PWR_TRIPLE_HOLD] == ACTION_TOGGLE_BATTLE_MODE);
  CHECK(cfg.action_map_on[SLOT_AUX_CLICK] == ACTION_BLAST);
  CHECK(cfg.action_map_on[SLOT_AUX_HOLD] == ACTION_LOCKUP_OR_DRAG);
  CHECK(cfg.action_map_on[SLOT_AUX_DOUBLE_HOLD] == ACTION_TOGGLE_MULTI_BLAST);
  CHECK(cfg.action_map_on[SLOT_PWR_AUX_CLICK] == ACTION_COLOR_CHANGE);
  CHECK(cfg.action_map_on[SLOT_PWR_MOD_STAB] == ACTION_MELT);
}

static void TestSa22cProfileDefaultsThreeButton() {
  RuntimeConfig cfg;
  cfg.SetDefaults();
  cfg.global.num_buttons = 3;
  ApplyButtonProfileDefaults(&cfg);

  CHECK(cfg.action_map_off[SLOT_AUX2_CLICK] == ACTION_PREV_PRESET);
  CHECK(cfg.action_map_on[SLOT_AUX2_HOLD] == ACTION_LIGHTNING_BLOCK);
  CHECK(cfg.action_map_on[SLOT_AUX2_DOUBLE_HOLD] == ACTION_TOGGLE_BATTLE_MODE);
}

static void TestResolveButtonSlotSa22cEvents() {
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_FIRST_SAVED_CLICK_SHORT, 0, 1) == SLOT_PWR_CLICK);
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_SECOND_SAVED_CLICK_SHORT, 0, 1) == SLOT_PWR_DOUBLE_CLICK);
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_THIRD_CLICK_SHORT, 0, 1) == SLOT_PWR_TRIPLE_CLICK);
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_FIRST_HELD_MEDIUM, 0, 2) == SLOT_PWR_HOLD_MEDIUM);
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_SECOND_HELD, 0, 2) == SLOT_PWR_DOUBLE_HOLD);
  CHECK(ResolveButtonSlot(BUTTON_POWER, EVENT_THIRD_HELD, 0, 2) == SLOT_PWR_TRIPLE_HOLD);
  CHECK(ResolveButtonSlot(BUTTON_AUX, EVENT_THIRD_CLICK_SHORT, 0, 2) == SLOT_AUX_TRIPLE_CLICK);
  CHECK(ResolveButtonSlot(BUTTON_AUX, EVENT_SECOND_HELD, 0, 2) == SLOT_AUX_DOUBLE_HOLD);
  CHECK(ResolveButtonSlot(BUTTON_AUX2, EVENT_SECOND_HELD, 0, 3) == SLOT_AUX2_DOUBLE_HOLD);
  CHECK(ResolveButtonSlot(BUTTON_NONE, EVENT_CLASH, BUTTON_POWER, 1) == SLOT_PWR_MOD_CLASH);
  CHECK(ResolveButtonSlot(BUTTON_NONE, EVENT_STAB, BUTTON_POWER, 1) == SLOT_PWR_MOD_STAB);
  CHECK(ResolveButtonSlot(BUTTON_NONE, EVENT_SWING, BUTTON_POWER, 1) == SLOT_PWR_MOD_SWING);
  CHECK(ResolveButtonSlot(BUTTON_NONE, EVENT_TWIST, BUTTON_POWER, 1) == SLOT_PWR_MOD_TWIST);
}

int main() {
  TestArgIndexConstants();
  TestStandardIncludesAllTuningArgs();
  TestNumericArgPositionsRemainStable();
  TestEveryMainStyleBuildContract();
  TestBaseContrastAliasAndClamps();
  TestStyleStringTruncationGuard();
  TestBladeBankSelectionRules();
  TestCopyGlobalAndActionsPreservesSourceValues();
  TestApplyButtonProfileDefaultsPopulatesPowerSlots();
  TestIniLoadSequenceReappliesButtonDefaults();
  TestBuildSaveDirPathForPresetsFile();
  TestSa22cProfileDefaultsOneButton();
  TestSa22cProfileDefaultsTwoButton();
  TestSa22cProfileDefaultsThreeButton();
  TestResolveButtonSlotSa22cEvents();
  return 0;
}
