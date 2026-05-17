# Available Blade Styles

Use these names in the `style` field of your preset sections in `saber_config.ini`.

## Main Blade Styles

| Style Name   | Description                              |
|-------------|------------------------------------------|
| standard    | Classic solid color blade                 |
| humpflicker | Subtle hump flicker blade                 |
| unstable    | Flickering/crackling unstable blade       |
| fire        | Flame animated blade                      |
| rainbow     | Cycling rainbow colors                    |
| strobe      | Strobing/flicker blade                    |
| pulse       | Pulsing glow blade                        |
| rotoscope   | Original trilogy rotoscope look           |
| ghostly     | Transparent/ethereal blade                |
| lightning   | Lightning/electricity animated            |
| darksaber   | Darksaber style (white core, dark edges)  |
| kylo        | Crossguard unstable variant               |
| prequels    | Prequel-era smooth blade                  |
| sequels     | Sequel-era with slight flicker            |
| ancient     | Ancient/Jedi temple style                 |

## Preset Off-Mode Controls

Each preset supports minimal off-state animation controls:

- `off_mode = pulse|random`
- `off_rate_ms = 10-60000`

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
