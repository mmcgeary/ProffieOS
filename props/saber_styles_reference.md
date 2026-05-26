# Style System Reference

This document describes the schema-driven style preset model used in
`saber_config.ini`. Both the firmware INI loader and the Companion app share
the same generated schema (`generated_style_schema`), so any style or
parameter listed here works identically in both.

## How INI Preset Keys Work

Every per-blade key is prefixed with `bladeN_` (1-based). When only one blade
is present you may omit the prefix (bare keys apply to blade 1).

```ini
[preset1]
font = bank/font1
blade1_style = standard
blade1_base_color = dodgerblue
blade1_ignition_time = 300
blade1_param.style_option = 2
```

### Two Syntax Paths

The firmware INI parser recognises two kinds of per-blade keys:

1. **Hardcoded keys** (`bladeN_<key>`) — a fixed set of keys parsed directly
   by the firmware. Only the keys listed in the tables below are accepted;
   any other `bladeN_<key>` form is **silently ignored**.
2. **Schema param keys** (`bladeN_param.<name>`) — arbitrary named parameters
   routed to the style template via `SetNamedStyleParam()`. These are defined
   by the generated style schema and may vary per style.

### Key Categories

| Category | Example key | Description |
|----------|-------------|-------------|
| Style selector | `blade1_style = standard` | Selects the style template |
| Hardcoded colors | `blade1_base_color = dodgerblue` | Fixed color keys accepted by the parser (see table below) |
| Hardcoded timing | `blade1_ignition_time = 300` | Fixed timing keys accepted by the parser |
| Runtime tuning | `blade1_flicker_depth = 8000` | Modulation knobs — universal across all styles (see matrix) |
| Schema params | `blade1_param.<name> = <value>` | Named style parameters defined by the schema |
| Preset-level | `off_mode`, `off_rate_ms`, `accent_style` | Shared across all blades in a preset |

## Main Blade Styles

| Style Name   | Description                              |
|-------------|------------------------------------------|
| standard    | Classic solid color blade                 |
| audioflicker| Audio-reactive flicker blade              |
| humpflicker | Subtle hump flicker blade                 |
| unstable    | Flickering/crackling unstable blade       |
| fire        | Flame animated blade                      |
| rainbow     | Cycling rainbow colors                    |
| strobe      | Strobing/flicker blade                    |
| pulse       | Pulsing glow blade                        |
| rotoscope   | Original trilogy rotoscope look           |
| ghostly     | Transparent/ethereal blade                |
| lightning   | Lightning/electricity animated            |
| darksaber   | Darksaber style (high contrast, dark edges)|
| kylo        | Crossguard unstable variant               |
| prequels    | Prequel-era smooth blade                  |
| sequels     | Sequel-era with slight flicker            |
| ancient     | Ancient/Jedi temple style                 |

## Hardcoded INI Keys (`bladeN_<key>`)

These keys are parsed directly by the firmware INI loader (`ParseBladeField` /
`ParseColorField`). They work with all styles and **do not** require the
`param.` prefix.

### Hardcoded Color Keys

| Key | Description |
|-----|-------------|
| `base_color` | Primary blade color |
| `alt_color` | Secondary / accent blade color |
| `blast_color` | Blaster deflect flash color |
| `clash_color` | Clash impact color |
| `lockup_color` | Lockup effect color |
| `drag_color` | Drag effect color |
| `lb_color` | Lightning block color |
| `stab_color` | Stab effect color |
| `swing_color` | Swing accent color |
| `emitter_color` | Emitter flare color |
| `preon_color` | Pre-on effect color |
| `off_color` | Off-state color |

### Hardcoded Timing Keys

| Key | Range | Description |
|-----|-------|-------------|
| `ignition_time` | 50–2000 | Ignition duration in ms |
| `retraction_time` | 50–2000 | Retraction duration in ms |

> **Note:** Other schema parameters such as `postoff_color`, `ignition_color`,
> `retraction_color`, `ignition_delay`, `ignition_power_up`,
> `retraction_delay`, `retraction_cool_down`, `lockup_position`, `drag_size`,
> `melt_size`, `emitter_size`, `preon_size`, and all `*_option` keys are
> **not** hardcoded. They must be set via `bladeN_param.<name>` (see below).

## Schema Parameters (`bladeN_param.<name>`)

All styles share a set of core schema parameters. These are routed through the
named-parameter mechanism and **must** use the `bladeN_param.<name>` syntax.

### Core Schema Params (all styles)

| Param key | Type | Description |
|-----------|------|-------------|
| `param.postoff_color` | color | Post-off color |
| `param.ignition_color` | color | Ignition flash color |
| `param.retraction_color` | color | Retraction flash color |
| `param.ignition_delay` | int | Ignition start delay |
| `param.ignition_power_up` | int | Ignition power-up option |
| `param.retraction_delay` | int | Retraction start delay |
| `param.retraction_cool_down` | int | Retraction cool-down option |
| `param.lockup_position` | int | Lockup effect position |
| `param.drag_size` | int | Drag effect size |
| `param.melt_size` | int | Melt effect size |
| `param.emitter_size` | int | Emitter flare size |
| `param.preon_size` | int | Pre-on effect size |
| `param.style_option` | int | Style variant selector |
| `param.swing_option` | int | Swing effect variant |
| `param.ignition_option` | int | Ignition animation variant |
| `param.retraction_option` | int | Retraction animation variant |
| `param.preon_option` | int | Pre-on animation variant |
| `param.off_option` | int | Off-state animation variant |

### Additional Params for `audioflicker`

The `audioflicker` style includes the secondary parameter set. These keys
are **only** accepted when the active style is `audioflicker` (or any future
style with `include_secondary: true` in the schema).

| Param key | Type | Description |
|-----------|------|-------------|
| `param.alt_color2` | color | Tertiary accent color |
| `param.alt_color3` | color | Quaternary accent color |
| `param.style_option2` | int | Second style variant selector |
| `param.style_option3` | int | Third style variant selector |
| `param.ignition_option2` | int | Second ignition animation variant |
| `param.retraction_option2` | int | Second retraction animation variant |

As more styles are added to the schema generator their style-specific
parameters will appear here automatically.

## Preset Off-Mode Controls

Each preset supports minimal off-state animation controls:

- `off_mode = pulse|random`
- `off_rate_ms = 10-60000`

## Runtime Tuning Parameters

These are **hardcoded** `bladeN_<key>` keys accepted by the INI parser for
every style. They feed the template style engine directly and control visual
modulation. While all styles accept these keys, most have a subtle (S) or no
(-) effect depending on the style (see matrix below).

| Key | Full range | Practical range | What it changes |
|-----|------------|-----------------|-----------------|
| flicker_depth | 0-32768 | 4000-18000 | Strength of base flicker modulation |
| flicker_speed | 1-20000 | 300-2500 | Flicker modulation rate |
| stripe_width | 1-65535 | 1500-12000 | Stripe band size in stripe-based styles |
| stripe_speed | 0-20000 | 100-3000 | Stripe movement speed in stripe-based styles |
| motion_gain | 0-32768 | 2500-14000 | Motion responsiveness for lockup/drag/melt shaping |
| noise_mix | 0-32768 | 2000-14000 | Brown-noise texture mix level |
| base_contrast | 0-32768 | 6000-28000 | Contrast between lit and dark portions of the base layer |
| pulse_rate | 1-20000 | 500-4000 | Pulse style cycle duration |
| pulse_depth | 0-32768 | 3000-18000 | Pulse style amplitude |
| strobe_freq | 1-200 | 8-60 | Strobe style frequency |
| strobe_ms | 1-1000 | 1-20 | Strobe style on-time in milliseconds |
| drift_rate | 0-32768 | 200-5000 | Hue-drift amount in base modulation |
| warm_shift | 0-32768 | 500-6000 | Warm color overlay amount |
| jitter_amount | 1-200 | 10-80 | Rapid jitter/strobe intensity |
| spark_mix | 0-32768 | 1500-12000 | Spark/emitter flicker contribution |
| heat_rand | 0-32768 | 1500-12000 | Fire/noise randomization level |
| fire_cooling | 0-255 | 20-120 | Fire cooling/ember variation |
| rainbow_speed | 1-20000 | 200-3000 | Rainbow style cycle speed |

`core_contrast` is still accepted as a legacy alias for backward compatibility, but `base_contrast` is the preferred key.

### Style x Parameter Matrix

Legend: **D** = dominant, **S** = subtle, **-** = ignored.

#### Style-Specific Parameters

| Style | stripe_width/speed | pulse_rate/depth | strobe_freq/ms | heat_rand/fire_cooling | rainbow_speed |
|-------|--------------------|------------------|----------------|------------------------|---------------|
| standard | - | - | - | - | - |
| humpflicker | - | - | - | - | - |
| unstable | - | - | S | - | - |
| fire | - | - | - | D | - |
| rainbow | - | - | - | - | D |
| strobe | - | - | D | - | - |
| pulse | - | D | - | - | - |
| rotoscope | D | - | - | - | - |
| ghostly | - | - | - | - | - |
| lightning | - | - | D | - | - |
| darksaber | D | - | - | - | - |
| kylo | - | - | S | - | - |
| prequels | - | - | - | - | - |
| sequels | - | - | - | - | - |
| ancient | D | - | - | - | - |

#### Shared Modulation Parameters (applies through primary blade skeleton)

| Style | flicker_depth/speed | motion_gain | noise_mix | base_contrast | drift_rate | warm_shift | jitter_amount | spark_mix |
|-------|----------------------|-------------|-----------|---------------|------------|------------|---------------|-----------|
| standard | S | S | S | S | S | S | S | S |
| humpflicker | S | S | S | D | S | S | S | S |
| unstable | D | S | D | S | S | S | D | D |
| fire | S | S | S | S | S | S | S | S |
| rainbow | S | S | S | S | S | S | S | S |
| strobe | S | S | S | S | S | S | D | S |
| pulse | S | S | S | S | S | S | S | S |
| rotoscope | S | S | S | D | S | S | S | S |
| ghostly | D | S | D | D | D | S | S | S |
| lightning | S | S | D | S | S | S | D | S |
| darksaber | S | S | S | D | S | S | S | S |
| kylo | D | S | D | S | S | S | D | D |
| prequels | S | S | S | S | S | S | S | S |
| sequels | S | S | S | S | S | S | S | S |
| ancient | S | S | S | D | S | D | S | S |

## Accent/Crystal Styles

For accent LEDs or crystal chambers, use `accent_style`:

| Style Name  | Description             |
|------------|-------------------------|
| pulse      | Slow pulsing glow       |
| blink      | On/off blinking         |
| random     | Random flickering       |
| static     | Always on (no animation)|
| color_cycle| Cycles through colors   |

Set `accent_speed` (100-10000 ms) to control animation timing.

## Color Names

All standard color names are supported (case-insensitive):

red, blue, green, white, cyan, yellow, orange, magenta, purple,
dodgerblue, deepskyblue, springgreen, gold, hotpink, coral,
tomato, salmon, violet, turquoise, steelblue, royalblue,
limegreen, chartreuse, lime, navy, teal, maroon, olive,
pink, plum, silver, skyblue, snow, ivory, lavender,
and many more (~66 total).

You can also use RGB tuples: `(255, 128, 0)` or `255,128,0`
Values are 0-255 per channel.

## Button Slot Reference

### 1-Button (slots 0-4)
| Slot | Trigger              |
|------|---------------------|
| 0    | POWER click          |
| 1    | POWER long click     |
| 2    | POWER hold           |
| 3    | POWER hold long      |
| 4    | POWER double click   |

### 2-Button adds (slots 5-12)
| Slot | Trigger              |
|------|---------------------|
| 5    | AUX click            |
| 6    | AUX long click       |
| 7    | AUX hold             |
| 8    | AUX hold long        |
| 9    | AUX double click     |
| 10   | POWER+AUX hold       |
| 11   | AUX+POWER hold       |
| 12   | POWER+AUX click      |

### 3-Button adds (slots 13-22)
| Slot | Trigger              |
|------|---------------------|
| 13   | AUX2 click           |
| 14   | AUX2 long click      |
| 15   | AUX2 hold            |
| 16   | AUX2 hold long       |
| 17   | AUX2 double click    |
| 18   | POWER+AUX2 hold      |
| 19   | AUX2+POWER hold      |
| 20   | AUX+AUX2 hold        |
| 21   | AUX2+AUX hold        |
| 22   | ALL three hold       |
