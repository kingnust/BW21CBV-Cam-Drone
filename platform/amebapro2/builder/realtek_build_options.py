import json
import subprocess
import sys
from pathlib import Path


MULTI_MODEL_MARKER = "BW21_NN_MODELS=yolov4_tiny,scrfd320p"
MULTI_MODEL_KEYS = ["yolov4_tiny", "scrfd320p"]


def packages_root(path_text):
    path = Path(path_text)
    while path.parent != path:
        if path.name.lower() == "packages":
            return path
        path = path.parent
    raise RuntimeError(f"No packages directory in hardware path: {path_text}")


def preserve_bw21_multi_model_manifest(options, roots, tools_path):
    sketch_path = Path(options.get("sketchLocation", ""))
    if not sketch_path.is_dir():
        return
    sketch_text = "\n".join(
        source.read_text(encoding="utf-8", errors="ignore")
        for source in sketch_path.glob("*.ino")
    )
    if MULTI_MODEL_MARKER not in sketch_text:
        return

    manifests = [tools_path / "amebapro2_fwfs_nn_models.json"]
    for root in roots:
        hardware_root = Path(root) / "realtek" / "hardware" / "AmebaPro2"
        if not hardware_root.is_dir():
            continue
        manifests.extend(hardware_root.glob(
            "*/variants/common_nn_models/amebapro2_fwfs_nn_models.json"
        ))
    manifests = [path for path in manifests if path.is_file()]
    if not manifests:
        raise RuntimeError("Could not locate the Realtek NN firmware manifest")

    for manifest_path in manifests:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        missing = [key for key in MULTI_MODEL_KEYS if key not in manifest]
        if missing:
            raise RuntimeError(
                "Realtek NN manifest is missing model key(s): " + ", ".join(missing)
            )
        manifest["FWFS"]["files"] = MULTI_MODEL_KEYS
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("Preserved BW21 NN models: " + ", ".join(MULTI_MODEL_KEYS))


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
    if result.returncode == 0 and model_source.lower() in ("flash", "loadfromflash"):
        preserve_bw21_multi_model_manifest(options, roots, tools_path)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
