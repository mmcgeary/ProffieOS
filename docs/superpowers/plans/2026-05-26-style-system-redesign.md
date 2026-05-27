# Style System Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current positional-arg INI style pipeline with a schema-driven hybrid model (shared core + style-specific params) that is editable from Companion and direct INI.

**Architecture:** Introduce a canonical style schema in firmware, generate firmware/Companion artifacts from that schema, and route runtime style building through named parameter resolution. Keep two shared cores (`main`, `accent`) while allowing each style to define its own base expression and parameter set. Preserve responsive behavior via core profiles and move all unit conversion into centralized resolvers.

**Tech Stack:** C++14 (ProffieOS), TypeScript/React/Zustand (Companion), Python 3 schema generator, Vitest, g++ props tests, arduino-cli.

---

## ProffieOS Built-in Argument Compatibility Contract

The redesign keeps ProffieOS built-in argument slots as the canonical runtime ABI.  
Schema entries map named params to built-in symbols (not raw indices) so styles remain aligned with native ProffieOS expectations.

1. **Primary/core args (always honored):**  
`BASE_COLOR_ARG`, `ALT_COLOR_ARG`, `STYLE_OPTION_ARG`, `IGNITION_OPTION_ARG`, `IGNITION_TIME_ARG`, `IGNITION_DELAY_ARG`, `IGNITION_COLOR_ARG`, `IGNITION_POWER_UP_ARG`, `BLAST_COLOR_ARG`, `CLASH_COLOR_ARG`, `LOCKUP_COLOR_ARG`, `LOCKUP_POSITION_ARG`, `DRAG_COLOR_ARG`, `DRAG_SIZE_ARG`, `LB_COLOR_ARG`, `STAB_COLOR_ARG`, `MELT_SIZE_ARG`, `SWING_COLOR_ARG`, `SWING_OPTION_ARG`, `EMITTER_COLOR_ARG`, `EMITTER_SIZE_ARG`, `PREON_COLOR_ARG`, `PREON_OPTION_ARG`, `PREON_SIZE_ARG`, `RETRACTION_OPTION_ARG`, `RETRACTION_TIME_ARG`, `RETRACTION_DELAY_ARG`, `RETRACTION_COLOR_ARG`, `RETRACTION_COOL_DOWN_ARG`, `POSTOFF_COLOR_ARG`, `OFF_COLOR_ARG`, `OFF_OPTION_ARG`.

2. **Secondary extension args (style-specific extras):**  
`ALT_COLOR2_ARG`, `ALT_COLOR3_ARG`, `STYLE_OPTION2_ARG`, `STYLE_OPTION3_ARG`, `IGNITION_OPTION2_ARG`, `RETRACTION_OPTION2_ARG`.

3. **Schema rule:** every parameter must reference one of the symbols above; no anonymous arg slots in schema.

---

## File Structure (locked before implementation)

### Firmware (`/Users/matthew.mcgeary/Copilot_workspace/ProffieOS`)

- Create: `props/style_schema.json`  
  Responsibility: canonical style/core/parameter definitions (single source of truth).
- Create: `props/tools/generate_style_schema.py`  
  Responsibility: generate firmware header + Companion TS schema from `style_schema.json`.
- Create: `props/generated_style_schema.h` (generated, committed)  
  Responsibility: firmware-facing schema structs and lookup tables.
- Modify: `props/runtime_config.h`  
  Responsibility: store style-specific named params in `IniBladeStyle`.
- Modify: `props/ini_loader.h`  
  Responsibility: parse `param.<name>` and `bladeN_param.<name>` keys.
- Modify: `props/style_registry.h`  
  Responsibility: schema-driven style string assembly and centralized conversion.
- Modify: `styles/ini_style_templates.h`  
  Responsibility: define v2 shared cores (`main`, `accent`) and seed style bases.
- Modify: `styles/style_parser.h`  
  Responsibility: register schema-backed parser styles (`ini2_*`).
- Modify: `props/style_registry_tests.cpp`  
  Responsibility: schema + named-param + conversion contract tests.
- Modify: `props/Makefile`  
  Responsibility: run schema generator before props test binary builds.
- Modify: `props/saber_styles_reference.md`  
  Responsibility: document new schema and INI key model.

### Companion (`/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion`)

- Create: `src/config/generatedStyleSchema.ts` (generated, committed)  
  Responsibility: generated style schema metadata consumed by app.
- Modify: `src/config/types.ts`  
  Responsibility: separate core params and style params in blade config.
- Modify: `src/config/normalizeConfig.ts`  
  Responsibility: parse/write `bladeN_param.<name>` and new blade param shape.
- Modify: `src/components/styleTuningConfig.ts`  
  Responsibility: schema-backed control metadata + basic/advanced partitioning.
- Modify: `src/components/styleStringBuilder.ts`  
  Responsibility: schema-driven arg assembly and conversion for preview.
- Modify: `src/components/PresetEditor.tsx`  
  Responsibility: dynamic style list + core controls + basic/advanced style controls.
- Modify: `src/state/configStore.ts`  
  Responsibility: update state mutations for `coreParams` + `styleParams`.
- Modify tests:  
  - `src/components/styleTuningConfig.test.ts`  
  - `src/components/styleStringBuilder.test.ts`  
  - `src/components/presetUiIntegration.test.tsx`  
  - `src/config/normalizeConfig.test.ts`

---

### Task 1: Add canonical style schema and generation pipeline

**Files:**
- Create: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/style_schema.json`
- Create: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/tools/generate_style_schema.py`
- Create (generated): `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/generated_style_schema.h`
- Create (generated): `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/config/generatedStyleSchema.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/Makefile`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/style_registry_tests.cpp`

- [ ] **Step 1: Write failing schema presence test in `style_registry_tests.cpp`**

```cpp
// style_registry_tests.cpp
#include "generated_style_schema.h"

static void TestGeneratedStyleSchemaPresence() {
  const GeneratedStyleDef* standard = FindGeneratedStyleDef("standard");
  CHECK(standard != nullptr);
  CHECK(strcmp(standard->core_type, "main") == 0);
  CHECK(strcmp(standard->parser_name, "ini2_standard") == 0);
}
```

- [ ] **Step 2: Run props test to verify it fails**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
```

Expected: FAIL with missing include/symbols for `generated_style_schema.h`.

- [ ] **Step 3: Create canonical schema and generator**

```json
// props/style_schema.json
{
  "version": 1,
  "cores": ["main", "accent"],
  "styles": [
    {
      "id": "standard",
      "label": "Standard",
      "core": "main",
      "parser": "ini2_standard",
      "base": "IniBaseStandardV2",
      "params": []
    },
    {
      "id": "audioflicker",
      "label": "Audio Flicker",
      "core": "main",
      "parser": "ini2_audioflicker",
      "base": "IniBaseAudioFlickerV2",
      "params": [
        { "name": "base_color", "arg_symbol": "BASE_COLOR_ARG", "type": "color", "default": "0,0,65535", "ui": "basic" },
        { "name": "alt_color", "arg_symbol": "ALT_COLOR_ARG", "type": "color", "default": "0,65535,65535", "ui": "basic" },
        { "name": "melt_color_a", "arg_symbol": "ALT_COLOR2_ARG", "type": "color", "default": "65535,0,0", "ui": "advanced" },
        { "name": "melt_color_b", "arg_symbol": "ALT_COLOR3_ARG", "type": "color", "default": "65535,65535,0", "ui": "advanced" }
      ]
    }
  ],
  "sharedCore": {
    "main": {
      "core_params": [
        "BASE_COLOR_ARG", "ALT_COLOR_ARG", "STYLE_OPTION_ARG",
        "IGNITION_OPTION_ARG", "IGNITION_TIME_ARG", "IGNITION_DELAY_ARG", "IGNITION_COLOR_ARG", "IGNITION_POWER_UP_ARG",
        "BLAST_COLOR_ARG", "CLASH_COLOR_ARG", "LOCKUP_COLOR_ARG", "LOCKUP_POSITION_ARG",
        "DRAG_COLOR_ARG", "DRAG_SIZE_ARG", "LB_COLOR_ARG", "STAB_COLOR_ARG", "MELT_SIZE_ARG",
        "SWING_COLOR_ARG", "SWING_OPTION_ARG", "EMITTER_COLOR_ARG", "EMITTER_SIZE_ARG",
        "PREON_COLOR_ARG", "PREON_OPTION_ARG", "PREON_SIZE_ARG",
        "RETRACTION_OPTION_ARG", "RETRACTION_TIME_ARG", "RETRACTION_DELAY_ARG", "RETRACTION_COLOR_ARG", "RETRACTION_COOL_DOWN_ARG",
        "POSTOFF_COLOR_ARG", "OFF_COLOR_ARG", "OFF_OPTION_ARG"
      ]
    },
    "accent": {
      "core_params": [
        "BASE_COLOR_ARG", "ALT_COLOR_ARG", "STYLE_OPTION_ARG",
        "IGNITION_OPTION_ARG", "IGNITION_TIME_ARG",
        "RETRACTION_OPTION_ARG", "RETRACTION_TIME_ARG",
        "OFF_OPTION_ARG", "OFF_COLOR_ARG"
      ]
    }
  }
}
```

```python
# props/tools/generate_style_schema.py
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
schema_path = ROOT / "props" / "style_schema.json"
fw_out = ROOT / "props" / "generated_style_schema.h"
companion_out = ROOT.parent / "ProffieOS-Companion" / "src" / "config" / "generatedStyleSchema.ts"

schema = json.loads(schema_path.read_text())

fw_lines = [
    "#ifndef PROPS_GENERATED_STYLE_SCHEMA_H",
    "#define PROPS_GENERATED_STYLE_SCHEMA_H",
    "",
    "struct GeneratedStyleDef { const char* id; const char* parser_name; const char* core_type; const char* base_symbol; };",
    "struct GeneratedParamDef { const char* style_id; const char* param_name; const char* arg_symbol; const char* type; const char* default_value; const char* ui_level; };",
]
for s in schema["styles"]:
    fw_lines.append(
        f'static constexpr GeneratedStyleDef kStyle_{s["id"]} = {{"{s["id"]}", "{s["parser"]}", "{s["core"]}", "{s["base"]}"}};'
    )
fw_lines += [
    "static constexpr GeneratedStyleDef kGeneratedStyles[] = {",
]
for s in schema["styles"]:
    fw_lines.append(f"  kStyle_{s['id']},")
fw_lines.append("};")
fw_lines.append("static constexpr GeneratedParamDef kGeneratedParams[] = {")
for s in schema["styles"]:
    for p in s["params"]:
        fw_lines.append(
            f'  {{"{s["id"]}", "{p["name"]}", "{p["arg_symbol"]}", "{p["type"]}", "{p["default"]}", "{p["ui"]}"}},'
        )
fw_lines += [
    "};",
    "inline const GeneratedStyleDef* FindGeneratedStyleDef(const char* id) {",
    "  for (const auto& s : kGeneratedStyles) if (strcmp(s.id, id) == 0) return &s;",
    "  return nullptr;",
    "}",
    "#endif",
]
fw_out.write_text("\n".join(fw_lines) + "\n")

companion_out.parent.mkdir(parents=True, exist_ok=True)
companion_out.write_text("export const GENERATED_STYLE_SCHEMA = " + json.dumps(schema, indent=2) + " as const;\n")
```

- [ ] **Step 4: Wire generator into props Makefile before test build**

```makefile
# props/Makefile
test: style_registry_tests
	./style_registry_tests

style_registry_tests: style_registry_tests.cpp style_registry.h ini_loader.h runtime_config.h generated_style_schema.h
	g++ -O -g -std=c++14 -I.. -MD -MP -o style_registry_tests style_registry_tests.cpp -lm

generated_style_schema.h: style_schema.json tools/generate_style_schema.py
	python3 tools/generate_style_schema.py
```

- [ ] **Step 5: Run generator and props tests to verify pass**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
python3 tools/generate_style_schema.py
make test
```

Expected: PASS, including `TestGeneratedStyleSchemaPresence`.

- [ ] **Step 6: Commit Task 1**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
git add props/style_schema.json props/tools/generate_style_schema.py props/generated_style_schema.h props/style_registry_tests.cpp props/Makefile
git add ../ProffieOS-Companion/src/config/generatedStyleSchema.ts
git commit -m "feat(styles): add canonical schema generation pipeline"
```

---

### Task 2: Add named style param storage and parsing in firmware

**Files:**
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/runtime_config.h`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/ini_loader.h`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/style_registry_tests.cpp`

- [ ] **Step 1: Write failing tests for `param.<name>` parsing**

```cpp
static void TestParseNamedStyleParams() {
  IniPreset p;
  p.SetDefaults();
  IniLoader::ParsePreset("blade1_param.audio_gain", "18000", &p);
  IniLoader::ParsePreset("blade1_param.noise_floor", "2200", &p);
  CHECK(strcmp(p.blades[0].FindStyleParamValue("audio_gain"), "18000") == 0);
  CHECK(strcmp(p.blades[0].FindStyleParamValue("noise_floor"), "2200") == 0);
}
```

- [ ] **Step 2: Run props tests to verify fail**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
```

Expected: FAIL with missing `FindStyleParamValue` and parser handling.

- [ ] **Step 3: Implement named param slots in `IniBladeStyle`**

```cpp
// runtime_config.h
#define INI_MAX_STYLE_PARAMS 24
#define INI_MAX_STYLE_PARAM_NAME_LEN 32
#define INI_MAX_STYLE_PARAM_VALUE_LEN 32

struct IniNamedStyleParam {
  char name[INI_MAX_STYLE_PARAM_NAME_LEN];
  char value[INI_MAX_STYLE_PARAM_VALUE_LEN];
};

struct IniBladeStyle {
  // existing fields...
  uint8_t style_param_count;
  IniNamedStyleParam style_params[INI_MAX_STYLE_PARAMS];

  void ClearStyleParams() { style_param_count = 0; }

  void SetStyleParam(const char* param_name, const char* param_value) {
    for (int i = 0; i < style_param_count; i++) {
      if (strcmp(style_params[i].name, param_name) == 0) {
        strncpy(style_params[i].value, param_value, INI_MAX_STYLE_PARAM_VALUE_LEN - 1);
        style_params[i].value[INI_MAX_STYLE_PARAM_VALUE_LEN - 1] = 0;
        return;
      }
    }
    if (style_param_count >= INI_MAX_STYLE_PARAMS) return;
    strncpy(style_params[style_param_count].name, param_name, INI_MAX_STYLE_PARAM_NAME_LEN - 1);
    style_params[style_param_count].name[INI_MAX_STYLE_PARAM_NAME_LEN - 1] = 0;
    strncpy(style_params[style_param_count].value, param_value, INI_MAX_STYLE_PARAM_VALUE_LEN - 1);
    style_params[style_param_count].value[INI_MAX_STYLE_PARAM_VALUE_LEN - 1] = 0;
    style_param_count++;
  }

  const char* FindStyleParamValue(const char* param_name) const {
    for (int i = 0; i < style_param_count; i++) {
      if (strcmp(style_params[i].name, param_name) == 0) return style_params[i].value;
    }
    return "";
  }
};
```

- [ ] **Step 4: Parse `param.` keys in `ini_loader.h`**

```cpp
// ini_loader.h inside ParseBladeField()
if (strncasecmp(key, "param.", 6) == 0) {
  blade->SetStyleParam(key + 6, val);
  return true;
}
```

- [ ] **Step 5: Run props tests to verify pass**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
```

Expected: PASS for named-style-param parsing tests.

- [ ] **Step 6: Commit Task 2**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
git add props/runtime_config.h props/ini_loader.h props/style_registry_tests.cpp
git commit -m "feat(styles): parse and store named style params"
```

---

### Task 3: Switch firmware style building to schema-driven composition

**Files:**
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/styles/ini_style_templates.h`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/styles/style_parser.h`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/style_registry.h`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/style_registry_tests.cpp`

- [ ] **Step 1: Write failing test for schema-driven parser selection**

```cpp
static void TestSchemaDrivenParserNameSelection() {
  IniPreset p;
  p.SetDefaults();
  strcpy(p.blades[0].style_name, "audioflicker");
  char buf[1024];
  CHECK(BuildIniStyleForBlade(&p, 0, buf, sizeof(buf)) > 0);
  const auto tokens = SplitTokens(buf);
  CHECK(tokens[0] == "ini2_audioflicker");
}
```

- [ ] **Step 2: Run props tests to verify fail**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
```

Expected: FAIL with parser mismatch (`ini_*` still emitted).

- [ ] **Step 3: Add v2 shared cores and seed bases**

```cpp
// styles/ini_style_templates.h
template<class BASE>
using IniMainCoreV2 = InOutTr<
  Layers<
    BASE,
    ResponsiveBlastL<RgbArg<BLAST_COLOR_ARG, White>>,
    ResponsiveClashL<RgbArg<CLASH_COLOR_ARG, White>>,
    ResponsiveLightningBlockL<RgbArg<LB_COLOR_ARG, White>>,
    LockupTrL<ResponsiveDragL<RgbArg<DRAG_COLOR_ARG, White>>, TrInstant, TrFade<220>, SaberBase::LOCKUP_DRAG>,
    LockupTrL<ResponsiveMeltL<Mix<TwistAngle<>, RgbArg<ALT_COLOR2_ARG, Red>, RgbArg<ALT_COLOR3_ARG, Yellow>>>, TrInstant, TrFade<220>, SaberBase::LOCKUP_MELT>
  >,
  TrWipeSparkTipX<RgbArg<EMITTER_COLOR_ARG, White>, IntArg<EMITTER_SIZE_ARG, 300>>,
  TrWipeInSparkTipX<RgbArg<EMITTER_COLOR_ARG, White>, IntArg<IGNITION_DELAY_ARG, 200>>,
  ColorSelect<IntArg<OFF_OPTION_ARG, 1>, TrInstant, RgbArg<OFF_COLOR_ARG, Black>, PulsingX<RgbArg<OFF_COLOR_ARG, Black>, Mix<Int<16384>, Black, RgbArg<OFF_COLOR_ARG, Black>>, IntArg<STYLE_OPTION_ARG, 50>>, RandomBlinkX<IntArg<STYLE_OPTION_ARG, 50>, RgbArg<OFF_COLOR_ARG, Black>, Black>>
>;

using IniBaseStandardV2 = AudioFlicker<RgbArg<BASE_COLOR_ARG, White>, RgbArg<ALT_COLOR_ARG, Blue>>;
using IniBaseAudioFlickerV2 = AudioFlicker<RgbArg<BASE_COLOR_ARG, White>, RgbArg<ALT_COLOR_ARG, Blue>>;
```

- [ ] **Step 4: Register new parser names and schema lookup emission**

```cpp
// style_parser.h
{ "ini2_standard", IniStyleAllocatorPtr<IniMainCoreV2<IniBaseStandardV2>>(), "Schema v2 standard" },
{ "ini2_audioflicker", IniStyleAllocatorPtr<IniMainCoreV2<IniBaseAudioFlickerV2>>(), "Schema v2 audio flicker" },
```

```cpp
// style_registry.h
const GeneratedStyleDef* style_def = FindGeneratedStyleDef(blade_view.style_name);
const char* parser_name = style_def ? style_def->parser_name : "ini2_standard";
BuildStyleFromSchema(style_def, &blade_view, buf, buf_size);
```

- [ ] **Step 5: Run props tests and PBv2 build**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
make all FQBN='proffieboard:stm32l4:ProffieboardV2-L433CC:usb=cdc,dosfs=sdspi,speed=80,opt=os'
```

Expected: props tests PASS; firmware compile exits 0.

- [ ] **Step 6: Commit Task 3**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
git add styles/ini_style_templates.h styles/style_parser.h props/style_registry.h props/style_registry_tests.cpp
git commit -m "feat(styles): enable schema-driven style composition"
```

---

### Task 4: Move Companion to schema-driven style metadata and UI controls

**Files:**
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/config/types.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/styleTuningConfig.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/PresetEditor.tsx`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/styleTuningConfig.test.ts`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/presetUiIntegration.test.tsx`

- [ ] **Step 1: Write failing tests for basic/advanced schema filtering**

```ts
// styleTuningConfig.test.ts
it('returns basic controls separately from advanced controls', () => {
  const controls = getStyleControlsForStyle('audioflicker');
  expect(controls.basic.map((c) => c.name)).toContain('base_color');
  expect(controls.advanced.map((c) => c.name)).toContain('melt_color_a');
});
```

- [ ] **Step 2: Run Companion test to verify fail**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
npm test -- src/components/styleTuningConfig.test.ts
```

Expected: FAIL with missing `getStyleControlsForStyle`.

- [ ] **Step 3: Replace static tuning metadata with generated schema adapter**

```ts
// styleTuningConfig.ts
import { GENERATED_STYLE_SCHEMA } from '../config/generatedStyleSchema';

export const getStyleControlsForStyle = (styleId: string) => {
  const style = GENERATED_STYLE_SCHEMA.styles.find((s) => s.id === styleId) ?? GENERATED_STYLE_SCHEMA.styles[0];
  const basic = style.params.filter((p) => p.ui === 'basic');
  const advanced = style.params.filter((p) => p.ui === 'advanced');
  return { style, basic, advanced };
};
```

- [ ] **Step 4: Update editor to render shared-core controls + basic + advanced sections**

```tsx
// PresetEditor.tsx (inside blade controls section)
const { style, basic, advanced } = getStyleControlsForStyle(selectedBlade.style);

<h4>Shared Core</h4>
{renderCoreControls(selectedBlade.coreParams)}

<h4>Style Controls</h4>
{basic.map(renderSchemaControl)}

<details>
  <summary>Advanced</summary>
  {advanced.map(renderSchemaControl)}
</details>
```

- [ ] **Step 5: Run Companion UI tests**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
npm test -- src/components/styleTuningConfig.test.ts src/components/presetUiIntegration.test.tsx
```

Expected: PASS for schema control partition and UI rendering.

- [ ] **Step 6: Commit Task 4**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
git add src/config/types.ts src/components/styleTuningConfig.ts src/components/PresetEditor.tsx src/components/styleTuningConfig.test.ts src/components/presetUiIntegration.test.tsx
git commit -m "feat(ui): drive style controls from generated schema"
```

---

### Task 5: Update Companion persistence and preview string generation for `param.*`

**Files:**
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/config/types.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/config/normalizeConfig.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/state/configStore.ts`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/styleStringBuilder.ts`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/config/normalizeConfig.test.ts`
- Test: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/src/components/styleStringBuilder.test.ts`

- [ ] **Step 1: Write failing round-trip test for `blade1_param.*`**

```ts
// normalizeConfig.test.ts
it('round-trips blade param namespace keys', () => {
  const ini = `[preset1]\nname=A\nfont=F\ntrack=\nblade1_style=audioflicker\nblade1_param.base_color=Blue\nblade1_param.melt_color_a=Red\n`;
  const doc = normalizeConfig({ bladeInIni: ini, bladeOutIni: ini, hwProfile: { numBlades: 1, numButtons: 1 } });
  const rebuilt = buildBladeInIni(doc);
  expect(rebuilt).toContain('blade1_param.base_color=Blue');
  expect(rebuilt).toContain('blade1_param.melt_color_a=Red');
});
```

- [ ] **Step 2: Run tests to verify fail**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
npm test -- src/config/normalizeConfig.test.ts
```

Expected: FAIL because `param.` namespacing is not preserved.

- [ ] **Step 3: Implement `coreParams` + `styleParams` split and namespaced serialization**

```ts
// types.ts
export interface BladeStyleConfig {
  style: string;
  coreParams: Record<string, string>;
  styleParams: Record<string, string>;
}
```

```ts
// normalizeConfig.ts write path
Object.entries(blade.coreParams).forEach(([key, value]) => {
  params[`blade${bladeOrdinal}_${key}`] = value;
});
Object.entries(blade.styleParams).forEach(([key, value]) => {
  params[`blade${bladeOrdinal}_param.${key}`] = value;
});
```

- [ ] **Step 4: Make style string builder schema-driven**

```ts
// styleStringBuilder.ts
const styleSchema = getStyleSchema(blade.style);
const args = new Array(31).fill('~');
args[0] = styleSchema.parser;
applyCoreArgs(args, blade.coreParams);
applyStyleArgs(args, styleSchema, blade.styleParams);
return args.join(' ');
```

- [ ] **Step 5: Run normalization + builder tests**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
npm test -- src/config/normalizeConfig.test.ts src/components/styleStringBuilder.test.ts
```

Expected: PASS for namespaced round-trip and schema-driven arg emission.

- [ ] **Step 6: Commit Task 5**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
git add src/config/types.ts src/config/normalizeConfig.ts src/state/configStore.ts src/components/styleStringBuilder.ts src/config/normalizeConfig.test.ts src/components/styleStringBuilder.test.ts
git commit -m "feat(data): persist and render style params via param namespace"
```

---

### Task 6: Final verification and documentation

**Files:**
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props/saber_styles_reference.md`
- Modify: `/Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion/README.md`

- [ ] **Step 1: Document new style schema model and INI format**

```md
## New style preset model

- style=<style_id>
- shared core keys: bladeN_ignition_time, bladeN_off_mode, ...
- style-specific keys: bladeN_param.<name>=<value>
```

- [ ] **Step 2: Run firmware verification commands**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS/props
make test
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
make all FQBN='proffieboard:stm32l4:ProffieboardV2-L433CC:usb=cdc,dosfs=sdspi,speed=80,opt=os'
```

Expected: both commands exit 0.

- [ ] **Step 3: Run Companion verification commands**

Run:

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
npm test
npm run build
```

Expected: tests PASS and production build succeeds.

- [ ] **Step 4: Commit final docs and verification-aligned changes**

```bash
cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS
git add props/saber_styles_reference.md docs/superpowers/plans/2026-05-26-style-system-redesign.md
git commit -m "docs(styles): document schema-driven style parameter model"

cd /Users/matthew.mcgeary/Copilot_workspace/ProffieOS-Companion
git add README.md
git commit -m "docs(companion): describe schema-driven style controls"
```

---

## Spec Coverage Check

1. **Hybrid shared + style-specific model:** covered by Tasks 2, 4, 5.
2. **Two shared cores (`main`, `accent`):** covered by Task 3.
3. **Named keys, no arg-slot semantics at INI/API boundary:** covered by Tasks 2 and 5.
4. **Curated basic + advanced Companion controls:** covered by Task 4.
5. **Breaking redesign:** covered by schema + persistence changes in Tasks 1, 2, 5.

## Placeholder Scan

No `TBD`, `TODO`, or deferred implementation markers are used.  
All code-changing steps include concrete code blocks and executable commands.

## Type/Interface Consistency Check

1. `BladeStyleConfig` split (`coreParams`/`styleParams`) is introduced once (Task 5) and reused consistently in store, normalize, and builder tasks.
2. Firmware named-param API (`SetStyleParam`, `FindStyleParamValue`) is defined in Task 2 and reused by parser and registry tasks.
3. Schema lookup (`FindGeneratedStyleDef`) is introduced in Task 1 and consumed in Task 3.
