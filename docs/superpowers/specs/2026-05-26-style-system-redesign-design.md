# Style System Redesign (INI + Companion + Firmware)

**Date:** 2026-05-26  
**Scope:** ProffieOS firmware + ProffieOS-Companion style parameterization contracts  
**Decision status:** Approved (design phase)

## 1. Problem Statement

The current INI style system achieved broad configurability, but style behavior became hard to reason about because:

1. A heavy shared wrapper can visually mask individual style intent.
2. Positional args (`arg3`, `arg4`, etc.) carry different meanings by style.
3. Unit conversions are distributed, making behavior less obvious.

Goal: preserve strong INI/Companion integration while making style behavior explicit, editor-aligned, and easy to tune.

## 2. Goals

1. Support user-supplied style-editor expressions as first-class library styles.
2. Expose editable visual controls via both Companion and direct INI editing.
3. Use **named parameters** (no generic arg-slot semantics).
4. Keep a **hybrid model**:
   - shared core controls for core behavior
   - style-specific controls for style identity
5. Use two shared cores:
   - `main` core for primary saber blades
   - `accent` core for crystal/accent blade behavior
6. Provide curated basic controls + advanced controls in Companion.
7. Use a clean, breaking redesign (no backward-compatibility layer).

## 3. Non-Goals

1. Preserving legacy style arg compatibility (`arg3/arg4` semantics).
2. Maintaining old preset-key compatibility.
3. Keeping old style wrappers if they conflict with style clarity.

## 4. Architecture

## 4.1 Style Definition Contract

Each library style is defined by a `StyleDefinition`:

1. `style_id` (stable identifier)
2. `label` (UI display name)
3. `core_type` (`main` | `accent`)
4. `base_expression` (style expression body derived from style editor source)
5. `parameters[]`:
   - `name`
   - `type` (`color`, `int`, `enum`, `bool`)
   - `default`
   - `constraints` (min/max/options)
   - `unit` + optional conversion metadata
   - `ui_group`
   - `ui_level` (`basic` | `advanced`)
6. `responsive_profile` (which shared core behavior stack to apply)

## 4.2 Composition Model

Final runtime style composition:

`Compose(core_type, responsive_profile, base_expression, resolved_named_params)`

This keeps style identity in `base_expression` and keeps behavior consistency in shared core profiles.

## 5. Data Model and IO

## 5.1 INI Structure

Per preset:

1. `style=<style_id>`
2. Shared-core keys remain explicit and human-editable (`ignition_time`, `retraction_time`, `off_mode`, `off_rate_ms`, `blast_color`, etc.).
3. Style-specific values are namespaced as `param.<name>` (for clarity and schema-driven parsing).

## 5.2 Single Source of Truth

Firmware owns canonical `StyleDefinition` and parameter schemas.  
Companion consumes generated schema metadata from the same source so:

1. labels/ranges/defaults stay aligned
2. basic vs advanced control grouping stays aligned
3. drift risk is minimized

## 5.3 Resolution Pipeline

At runtime:

1. Parse shared-core values.
2. Parse `param.*` into a style-param map.
3. Resolve selected style schema:
   - load value from INI if present
   - clamp and convert units
   - fallback to schema default
4. Compose final style from resolved params.

All conversions (example: ms -> RPM where required) happen in one resolver layer.

## 6. Companion UX Contract

1. Companion binds controls from schema, not hardcoded per-style assumptions.
2. Basic panel shows curated controls (`ui_level=basic`).
3. Advanced panel exposes full style-specific controls.
4. Switching styles rebinds controls to the selected style schema.

## 7. Style Intake Workflow (User-Supplied Styles)

For each provided editor style expression:

1. Assign `style_id` and `core_type`.
2. Convert editable literals into named parameters.
3. Keep non-editable literals fixed in the base expression.
4. Classify each parameter as basic vs advanced.
5. Define defaults, ranges, and units.
6. Register schema and verify firmware + Companion parity.

## 8. Validation Requirements

1. Schema compile checks:
   - every referenced param is declared
   - every declared param has defaults and constraints
2. INI round-trip tests:
   - `param.*` parse, clamp, and persist correctly
3. Companion contract tests:
   - control generation matches schema
   - basic/advanced grouping is correct
4. Runtime composition tests:
   - style outputs resolve expected defaults
   - conversion paths are deterministic

## 9. Breaking Redesign Policy

This redesign intentionally does **not** preserve legacy style key/arg compatibility.  
New style library and parameter contracts are canonical going forward.

## 10. Open Inputs Needed From User

1. Ordered list of initial style-editor expressions to onboard first.
2. Preferred naming for style IDs (short IDs vs descriptive IDs).
3. Preferred initial split of shared-core controls between `main` and `accent`.
