#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROPS_DIR = SCRIPT_DIR.parent
REPO_ROOT = PROPS_DIR.parent
DEFAULT_SCHEMA_PATH = PROPS_DIR / "style_schema.json"
DEFAULT_HEADER_PATH = PROPS_DIR / "generated_style_schema.h"


def escape_c_string(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


def normalize_params(params, context):
    normalized = []
    for index, param in enumerate(params):
        if not isinstance(param, dict):
            raise ValueError(f"{context}[{index}] must be an object")
        key = param.get("key")
        arg_symbol = param.get("arg_symbol")
        if not isinstance(key, str) or not key:
            raise ValueError(f"{context}[{index}].key must be a non-empty string")
        if not isinstance(arg_symbol, str) or not arg_symbol:
            raise ValueError(f"{context}[{index}].arg_symbol must be a non-empty string")
        normalized.append({"key": key, "arg_symbol": arg_symbol})
    return normalized


def merge_params(*param_lists):
    merged = []
    seen_symbols = set()
    for params in param_lists:
        for param in params:
            symbol = param["arg_symbol"]
            if symbol in seen_symbols:
                continue
            seen_symbols.add(symbol)
            merged.append(param)
    return merged


def load_schema(schema_path):
    with schema_path.open("r", encoding="utf-8") as file:
        schema = json.load(file)

    if not isinstance(schema, dict):
        raise ValueError("style_schema.json must contain an object")
    for required_key in ("version", "cores", "sharedCore", "styles"):
        if required_key not in schema:
            raise ValueError(f"Missing required key: {required_key}")

    if not isinstance(schema["version"], int):
        raise ValueError("version must be an integer")
    if not isinstance(schema["cores"], dict):
        raise ValueError("cores must be an object")
    if not isinstance(schema["sharedCore"], dict):
        raise ValueError("sharedCore must be an object")
    if not isinstance(schema["styles"], list):
        raise ValueError("styles must be an array")

    return schema


def build_generated_tables(schema):
    cores = schema["cores"]
    shared_core = schema["sharedCore"]

    shared_params = {}
    for shared_name, shared_definition in shared_core.items():
        if not isinstance(shared_definition, dict):
            raise ValueError(f"sharedCore.{shared_name} must be an object")
        params = shared_definition.get("params", [])
        if not isinstance(params, list):
            raise ValueError(f"sharedCore.{shared_name}.params must be an array")
        shared_params[shared_name] = normalize_params(params, f"sharedCore.{shared_name}.params")

    flat_params = []
    style_defs = []

    for index, style in enumerate(schema["styles"]):
        if not isinstance(style, dict):
            raise ValueError(f"styles[{index}] must be an object")

        name = style.get("name")
        core_name = style.get("core")
        parser_name = style.get("parser_name")
        include_secondary = bool(style.get("include_secondary", False))

        if not isinstance(name, str) or not name:
            raise ValueError(f"styles[{index}].name must be a non-empty string")
        if not isinstance(core_name, str) or not core_name:
            raise ValueError(f"styles[{index}].core must be a non-empty string")
        if not isinstance(parser_name, str) or not parser_name:
            raise ValueError(f"styles[{index}].parser_name must be a non-empty string")
        if core_name not in cores:
            raise ValueError(f"styles[{index}].core references unknown core '{core_name}'")

        core_definition = cores[core_name]
        if not isinstance(core_definition, dict):
            raise ValueError(f"cores.{core_name} must be an object")
        core_type = core_definition.get("core_type", core_name)
        if not isinstance(core_type, str) or not core_type:
            raise ValueError(f"cores.{core_name}.core_type must be a non-empty string")

        if "params" in style and not isinstance(style["params"], list):
            raise ValueError(f"styles[{index}].params must be an array")
        style_specific_params = normalize_params(style.get("params", []), f"styles[{index}].params")

        core_shared_params = shared_params.get(core_name, [])
        secondary_params = shared_params.get("secondary", []) if include_secondary else []
        merged_params = merge_params(core_shared_params, secondary_params, style_specific_params)

        param_offset = len(flat_params)
        for param in merged_params:
            flat_params.append(
                {"style_name": name, "key": param["key"], "arg_symbol": param["arg_symbol"]}
            )

        style_defs.append(
            {
                "name": name,
                "core_type": core_type,
                "parser_name": parser_name,
                "param_offset": param_offset,
                "param_count": len(merged_params),
            }
        )

    return flat_params, style_defs


def render_header(schema_version, flat_params, style_defs):
    lines = []
    lines.append("// AUTO-GENERATED by props/tools/generate_style_schema.py. DO NOT EDIT.")
    lines.append("#ifndef PROPS_GENERATED_STYLE_SCHEMA_H")
    lines.append("#define PROPS_GENERATED_STYLE_SCHEMA_H")
    lines.append("")
    lines.append("#include <cstring>")
    lines.append("")
    lines.append("struct GeneratedParamDef {")
    lines.append("  const char* style_name;")
    lines.append("  const char* key;")
    lines.append("  const char* arg_symbol;")
    lines.append("};")
    lines.append("")
    lines.append("struct GeneratedStyleDef {")
    lines.append("  const char* name;")
    lines.append("  const char* core_type;")
    lines.append("  const char* parser_name;")
    lines.append("  int param_offset;")
    lines.append("  int param_count;")
    lines.append("};")
    lines.append("")
    lines.append(f"static constexpr int kGeneratedStyleSchemaVersion = {schema_version};")
    lines.append("")
    lines.append("static const GeneratedParamDef kGeneratedParamDefs[] = {")
    for param in flat_params:
        lines.append(
            '  {"%s", "%s", "%s"},'
            % (
                escape_c_string(param["style_name"]),
                escape_c_string(param["key"]),
                escape_c_string(param["arg_symbol"]),
            )
        )
    lines.append("};")
    lines.append("")
    lines.append("static constexpr int kGeneratedParamDefCount =")
    lines.append("    static_cast<int>(sizeof(kGeneratedParamDefs) / sizeof(kGeneratedParamDefs[0]));")
    lines.append("")
    lines.append("static const GeneratedStyleDef kGeneratedStyleDefs[] = {")
    for style in style_defs:
        lines.append(
            '  {"%s", "%s", "%s", %d, %d},'
            % (
                escape_c_string(style["name"]),
                escape_c_string(style["core_type"]),
                escape_c_string(style["parser_name"]),
                style["param_offset"],
                style["param_count"],
            )
        )
    lines.append("};")
    lines.append("")
    lines.append("static constexpr int kGeneratedStyleDefCount =")
    lines.append("    static_cast<int>(sizeof(kGeneratedStyleDefs) / sizeof(kGeneratedStyleDefs[0]));")
    lines.append("")
    lines.append("static inline const GeneratedStyleDef* FindGeneratedStyleDef(const char* style_name) {")
    lines.append("  if (!style_name) {")
    lines.append("    return nullptr;")
    lines.append("  }")
    lines.append("  for (int i = 0; i < kGeneratedStyleDefCount; ++i) {")
    lines.append("    if (std::strcmp(style_name, kGeneratedStyleDefs[i].name) == 0) {")
    lines.append("      return &kGeneratedStyleDefs[i];")
    lines.append("    }")
    lines.append("  }")
    lines.append("  return nullptr;")
    lines.append("}")
    lines.append("")
    lines.append("#endif  // PROPS_GENERATED_STYLE_SCHEMA_H")
    lines.append("")
    return "\n".join(lines)


def render_companion_schema(schema):
    pretty_json = json.dumps(schema, indent=2, ensure_ascii=False)
    return (
        "// AUTO-GENERATED by props/tools/generate_style_schema.py. DO NOT EDIT.\n"
        "export const generatedStyleSchema = "
        f"{pretty_json} as const;\n"
    )


def resolve_companion_output_path(repo_root):
    candidates = []

    if repo_root.parent.name == ".worktrees":
        worktree_name = repo_root.name
        workspace_root = repo_root.parents[2]
        candidates.append(
            workspace_root
            / "ProffieOS-Companion"
            / ".worktrees"
            / worktree_name
            / "src"
            / "config"
            / "generatedStyleSchema.ts"
        )

    candidates.extend(
        [
            repo_root.parent / "ProffieOS-Companion" / "src" / "config" / "generatedStyleSchema.ts",
            repo_root.parents[1] / "ProffieOS-Companion" / "src" / "config" / "generatedStyleSchema.ts",
        ]
    )

    for candidate in candidates:
        if candidate.parent.exists():
            return candidate
    raise FileNotFoundError("Unable to locate ProffieOS-Companion/src/config directory")


def write_if_changed(path, content):
    if path.exists():
        existing = path.read_text(encoding="utf-8")
        if existing == content:
            return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description="Generate style schema artifacts")
    parser.add_argument("--schema", default=str(DEFAULT_SCHEMA_PATH))
    parser.add_argument("--header-output", default=str(DEFAULT_HEADER_PATH))
    parser.add_argument("--companion-output", default=None)
    return parser.parse_args()


def main():
    args = parse_args()

    schema_path = Path(args.schema).resolve()
    header_output_path = Path(args.header_output).resolve()
    companion_output_path = (
        Path(args.companion_output).resolve()
        if args.companion_output
        else resolve_companion_output_path(REPO_ROOT)
    )

    schema = load_schema(schema_path)
    flat_params, style_defs = build_generated_tables(schema)

    header_content = render_header(schema["version"], flat_params, style_defs)
    companion_content = render_companion_schema(schema)

    write_if_changed(header_output_path, header_content)
    write_if_changed(companion_output_path, companion_content)


if __name__ == "__main__":
    main()
