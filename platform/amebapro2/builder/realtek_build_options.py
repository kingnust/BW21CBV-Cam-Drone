import json
import subprocess
import sys
from pathlib import Path


def packages_root(path_text):
    path = Path(path_text)
    while path.parent != path:
        if path.name.lower() == "packages":
            return path
        path = path.parent
    raise RuntimeError(f"No packages directory in hardware path: {path_text}")


def main():
    if len(sys.argv) != 5:
        raise RuntimeError(
            "Expected build path, Realtek tools path, model source, and tool name"
        )

    build_path = Path(sys.argv[1])
    tools_path = Path(sys.argv[2])
    model_source = sys.argv[3]
    tool_name = sys.argv[4]
    options_path = build_path / "build.options.json"
    options = json.loads(options_path.read_text(encoding="utf-8"))
    hardware_paths = [
        value.strip()
        for value in options.get("hardwareFolders", "").split(",")
        if value.strip()
    ]
    roots = []
    for hardware_path in hardware_paths:
        root = str(packages_root(hardware_path))
        if root.lower() not in (item.lower() for item in roots):
            roots.append(root)
    if not roots:
        raise RuntimeError("Arduino CLI produced no hardwareFolders entries")

    options["hardwareFolders"] = ",".join(roots)
    options_path.write_text(json.dumps(options, indent=2) + "\n", encoding="utf-8")

    tool = tools_path / tool_name
    result = subprocess.run(
        [str(tool), str(build_path), str(tools_path), model_source], check=False
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
