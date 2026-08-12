import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
import zipfile
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from SCons.Script import Default, DefaultEnvironment

env = DefaultEnvironment()

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
BUILD_DIR = Path(env.subst("$BUILD_DIR"))
SRC_DIR = Path(env.subst("$PROJECT_SRC_DIR"))
PROJECT_KEY = hashlib.sha1(
    f"{PROJECT_DIR}:{env.subst('$PIOENV')}".encode("utf-8")
).hexdigest()[:10]
TOOLS_DIR = Path.home() / ".bw21-platformio"
WORK_DIR = TOOLS_DIR / "work" / PROJECT_KEY
LOCAL_APP_DATA = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
ARDUINO_DATA_DIR = LOCAL_APP_DATA / "Arduino15"
ARDUINO_DOWNLOADS_DIR = TOOLS_DIR / "arduino-downloads"
ARDUINO_USER_DIR = TOOLS_DIR / "arduino-user"
ARDUINO_CONFIG = TOOLS_DIR / "arduino-cli.yaml"
SKETCH_DIR = WORK_DIR / "sketch" / "BW21Cam"
ARDUINO_BUILD_DIR = WORK_DIR / "arduino-build"
ARDUINO_OUTPUT_DIR = BUILD_DIR / "firmware"
BUILD_STAMP = BUILD_DIR / "bw21-build.json"
REALTEK_TOOL_LOCK = TOOLS_DIR / "realtek-tools.lock"
CLI_COMPAT_HELPER = (
    Path(env.subst("$BUILD_SCRIPT")).resolve().with_name("realtek_build_options.py")
)

CLI_VERSION = env.GetProjectOption("custom_arduino_cli_version")
CORE_SPEC = env.GetProjectOption("custom_amebapro2_core")
PACKAGE_INDEX = env.GetProjectOption("custom_amebapro2_index")
FQBN = env.GetProjectOption("custom_amebapro2_fqbn")
FC_LINK = env.GetProjectOption("custom_fc_link", "0")
ONDEVICE_VISION = env.GetProjectOption("custom_ondevice_vision", "0")

NN_MODELS_VERSION = "1.0.3"
NN_MODELS_ARCHIVE = f"ameba_pro2_nn_models-{NN_MODELS_VERSION}.tar.gz"
NN_MODELS_URL = (
    "https://github.com/Ameba-AIoT/ameba-arduino-pro2/raw/main/"
    f"Arduino_package/release/{NN_MODELS_ARCHIVE}"
)
NN_MODELS_SHA256 = "acb3c512fe8e84531409b6cd4954dee84a63f7cdb9529479699af881b8fbf067"
NN_MODEL_KEYS = ["yolov4_tiny"]
MIN_FIRMWARE_BYTES_WITH_MODEL = 4 * 1024 * 1024
REALTEK_TOOL_LOCK_TIMEOUT_SECONDS = 15 * 60


def cli_executable():
    executable = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    return TOOLS_DIR / "arduino-cli" / executable


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@contextmanager
def realtek_tool_lock():
    REALTEK_TOOL_LOCK.parent.mkdir(parents=True, exist_ok=True)
    with REALTEK_TOOL_LOCK.open("a+b") as lock_file:
        lock_file.seek(0, os.SEEK_END)
        if lock_file.tell() == 0:
            lock_file.write(b"\0")
            lock_file.flush()

        deadline = time.monotonic() + REALTEK_TOOL_LOCK_TIMEOUT_SECONDS
        while True:
            try:
                lock_file.seek(0)
                if os.name == "nt":
                    import msvcrt

                    msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Timed out waiting for the shared Realtek tools")
                time.sleep(0.25)

        try:
            yield
        finally:
            lock_file.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def write_cli_config():
    for directory in (
        TOOLS_DIR,
        ARDUINO_DATA_DIR,
        ARDUINO_DOWNLOADS_DIR,
        ARDUINO_USER_DIR,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    def yaml_path(path):
        return Path(os.path.abspath(path)).as_posix()

    ARDUINO_CONFIG.write_text(
        "board_manager:\n"
        "  additional_urls:\n"
        f"    - {PACKAGE_INDEX}\n"
        "directories:\n"
        f"  data: {yaml_path(ARDUINO_DATA_DIR)}\n"
        f"  downloads: {yaml_path(ARDUINO_DOWNLOADS_DIR)}\n"
        f"  user: {yaml_path(ARDUINO_USER_DIR)}\n",
        encoding="ascii",
    )


def cli_archive():
    system = platform.system().lower()
    machine = platform.machine().lower()
    if system == "windows" and machine in ("amd64", "x86_64"):
        return f"arduino-cli_{CLI_VERSION}_Windows_64bit.zip"
    if system == "linux" and machine in ("amd64", "x86_64"):
        return f"arduino-cli_{CLI_VERSION}_Linux_64bit.tar.gz"
    if system == "darwin" and machine in ("arm64", "aarch64"):
        return f"arduino-cli_{CLI_VERSION}_macOS_ARM64.tar.gz"
    if system == "darwin" and machine in ("amd64", "x86_64"):
        return f"arduino-cli_{CLI_VERSION}_macOS_64bit.tar.gz"
    raise RuntimeError(f"Unsupported host for automatic Arduino CLI setup: {system}/{machine}")


def install_cli():
    archive_name = cli_archive()
    archive_path = TOOLS_DIR / archive_name
    destination = cli_executable().parent
    destination.mkdir(parents=True, exist_ok=True)
    url = (
        f"https://github.com/arduino/arduino-cli/releases/download/"
        f"v{CLI_VERSION}/{archive_name}"
    )
    print(f"Downloading Arduino CLI {CLI_VERSION}...")
    urllib.request.urlretrieve(url, archive_path)
    if archive_name.endswith(".zip"):
        with zipfile.ZipFile(archive_path) as archive:
            archive.extractall(destination)
    else:
        with tarfile.open(archive_path, "r:gz") as archive:
            archive.extractall(destination)
    archive_path.unlink(missing_ok=True)
    if not cli_executable().is_file():
        raise RuntimeError("Arduino CLI archive did not contain the expected executable")
    if os.name != "nt":
        cli_executable().chmod(0o755)


def patch_ameba_wifi_send_result(core_version):
    if not core_version:
        raise RuntimeError("The AmebaPro2 core version must be pinned")

    core_dir = (
        ARDUINO_DATA_DIR
        / "packages"
        / "realtek"
        / "hardware"
        / "AmebaPro2"
        / core_version
    )
    patches = (
        (
            core_dir / "cores" / "ambpro2" / "server_drv.h",
            "bool sendData(int sock, const uint8_t *data, uint32_t len);",
            "int sendData(int sock, const uint8_t *data, uint32_t len);",
        ),
        (
            core_dir / "cores" / "ambpro2" / "server_drv.cpp",
            "bool ServerDrv::sendData(int sock, const uint8_t *data, uint32_t len)",
            "int ServerDrv::sendData(int sock, const uint8_t *data, uint32_t len)",
        ),
    )

    for path, original, corrected in patches:
        content = path.read_text(encoding="utf-8")
        if corrected in content:
            continue
        if original not in content:
            raise RuntimeError(f"Unsupported AmebaPro2 WiFi API in {path}")
        path.write_text(content.replace(original, corrected, 1), encoding="utf-8")
        print(f"Patched AmebaPro2 TCP send result: {path.name}")


def patch_ameba_cli_compatibility(core_version):
    platform_path = (
        ARDUINO_DATA_DIR
        / "packages"
        / "realtek"
        / "hardware"
        / "AmebaPro2"
        / core_version
        / "platform.txt"
    )
    hooks = (
        ("3", "ino_validation_windows.exe"),
        ("5", "nn_json_modify_windows.exe"),
    )
    content = platform_path.read_text(encoding="utf-8")
    changed = False
    for number, tool_name in hooks:
        prefix = f"recipe.hooks.prebuild.{number}.pattern.windows = "
        original = (
            prefix + f'"{{ameba.tools_path}}/{tool_name}" "{{build.path}}" '
            '"{ameba.tools_path}" "{build.model_src}"'
        )
        corrected = (
            prefix + f'"{sys.executable}" "{CLI_COMPAT_HELPER}" "{{build.path}}" '
            f'"{{ameba.tools_path}}" "{{build.model_src}}" "{tool_name}"'
        )
        if corrected in content:
            continue
        installed = next(
            (line for line in content.splitlines() if line.startswith(prefix)), None
        )
        is_known_helper = installed is not None and "realtek_build_options.py" in installed
        has_expected_arguments = installed is not None and (
            installed.endswith(f'"{{build.model_src}}" "{tool_name}"')
            or installed.endswith('"{build.model_src}"')
        )
        if installed != original and not (is_known_helper and has_expected_arguments):
            raise RuntimeError(
                f"Unsupported AmebaPro2 prebuild hook {number} in {platform_path}"
            )
        content = content.replace(installed, corrected, 1)
        changed = True
    if changed:
        platform_path.write_text(content, encoding="utf-8")
        print("Patched AmebaPro2 Arduino CLI model-packaging compatibility")


def select_nn_models(destination):
    manifest_path = destination / "common_nn_models" / "amebapro2_fwfs_nn_models.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if "FWFS" not in manifest or not isinstance(manifest["FWFS"], dict):
        raise RuntimeError("The Realtek NN model manifest has no FWFS profile")
    missing = [key for key in NN_MODEL_KEYS if key not in manifest]
    if missing:
        raise RuntimeError("Unknown Realtek NN model key(s): " + ", ".join(missing))
    manifest["FWFS"]["files"] = NN_MODEL_KEYS
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def ensure_nn_models():
    destination = (
        ARDUINO_DATA_DIR
        / "packages"
        / "realtek"
        / "tools"
        / "ameba_pro2_nn_models"
        / NN_MODELS_VERSION
    )
    required = destination / "common_nn_models" / "yolov4_tiny.nb"
    if required.is_file():
        select_nn_models(destination)
        return

    package_dir = ARDUINO_DOWNLOADS_DIR / "packages"
    package_dir.mkdir(parents=True, exist_ok=True)
    archive_path = package_dir / NN_MODELS_ARCHIVE
    if not archive_path.is_file():
        print(f"Downloading official Realtek NN models {NN_MODELS_VERSION}...")
        urllib.request.urlretrieve(NN_MODELS_URL, archive_path)

    digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    if digest != NN_MODELS_SHA256:
        archive_path.unlink(missing_ok=True)
        raise RuntimeError("The downloaded Realtek NN model archive failed SHA-256 verification")

    staging = destination.parent / f".{NN_MODELS_VERSION}.installing"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    try:
        with tarfile.open(archive_path, "r:gz") as archive:
            staging_root = staging.resolve()
            for member in archive.getmembers():
                extracted = (staging / member.name).resolve()
                if staging_root not in extracted.parents and extracted != staging_root:
                    raise RuntimeError("Unsafe path in Realtek NN model archive")
            archive.extractall(staging)

        extracted_root = staging / "ameba_pro2_nn_models"
        if not (extracted_root / "common_nn_models" / "yolov4_tiny.nb").is_file():
            raise RuntimeError("Realtek NN archive did not contain YOLOv4-tiny")
        if destination.exists():
            shutil.rmtree(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(extracted_root), str(destination))
        select_nn_models(destination)
        print(f"Staged official Realtek NN models {NN_MODELS_VERSION}")
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def verify_vision_image(flash_path, core_version):
    if ONDEVICE_VISION != "1":
        return None
    if not flash_path.is_file():
        raise RuntimeError(f"Expected vision image was not produced: {flash_path}")

    model_root = (
        ARDUINO_DATA_DIR
        / "packages"
        / "realtek"
        / "hardware"
        / "AmebaPro2"
        / core_version
        / "variants"
        / "common_nn_models"
    )
    manifest_path = model_root / "amebapro2_fwfs_nn_models.json"
    model_path = model_root / "yolov4_tiny.nb"
    if not manifest_path.is_file() or not model_path.is_file():
        raise RuntimeError("The compiled image has no verifiable Realtek YOLO model source")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = manifest.get("FWFS", {}).get("files")
    if selected != NN_MODEL_KEYS:
        raise RuntimeError(
            f"The Realtek firmware manifest selected {selected!r}, expected {NN_MODEL_KEYS!r}"
        )

    model_bytes = model_path.stat().st_size
    image_bytes = flash_path.stat().st_size
    minimum_bytes = model_bytes + MIN_FIRMWARE_BYTES_WITH_MODEL
    if image_bytes < minimum_bytes:
        raise RuntimeError(
            "Vision firmware is too small to contain YOLOv4-tiny: "
            f"image={image_bytes}, model={model_bytes}, minimum={minimum_bytes}"
        )
    print(
        "Verified packaged YOLOv4-tiny: "
        f"model={model_bytes} bytes, flash={image_bytes} bytes"
    )
    return {
        "model": NN_MODEL_KEYS[0],
        "model_bytes": model_bytes,
        "flash_bytes": image_bytes,
    }


def run_cli(arguments, capture=False, show_captured_output=True):
    command = [
        str(cli_executable()),
        "--config-file",
        str(ARDUINO_CONFIG),
        *arguments,
    ]
    print("$ " + " ".join(command))
    result = subprocess.run(
        command,
        check=False,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )
    if capture and result.stdout and (show_captured_output or result.returncode != 0):
        print(result.stdout, end="")
    if result.returncode != 0:
        raise RuntimeError(f"Arduino CLI failed with exit code {result.returncode}")
    return result.stdout or ""


def ensure_toolchain():
    write_cli_config()
    if not cli_executable().is_file():
        install_cli()

    core_id, _, required_version = CORE_SPEC.partition("@")
    installed = run_cli(
        ["core", "list", "--format", "json"],
        capture=True,
        show_captured_output=False,
    )
    try:
        parsed_cores = json.loads(installed or "[]")
        installed_cores = (
            parsed_cores.get("platforms", [])
            if isinstance(parsed_cores, dict)
            else parsed_cores
        )
    except json.JSONDecodeError:
        installed_cores = []

    found = False
    for item in installed_cores:
        item_id = item.get("id") or item.get("ID")
        item_version = (
            item.get("installed_version")
            or item.get("installed")
            or item.get("Installed")
        )
        if item_id == core_id and (not required_version or item_version == required_version):
            found = True
            break

    if not found:
        print(f"Installing official AmebaPro2 core {CORE_SPEC}...")
        run_cli(["core", "update-index"])
        run_cli(["core", "install", CORE_SPEC])

    patch_ameba_wifi_send_result(required_version)
    patch_ameba_cli_compatibility(required_version)


def realtek_uploader_directory(core_version):
    for index_path in sorted(ARDUINO_DATA_DIR.glob("package*.json")):
        try:
            index = json.loads(index_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        for package in index.get("packages", []):
            if package.get("name") != "realtek":
                continue
            for installed_platform in package.get("platforms", []):
                if (
                    installed_platform.get("architecture") != "AmebaPro2"
                    or installed_platform.get("version") != core_version
                ):
                    continue
                for dependency in installed_platform.get("toolsDependencies", []):
                    if dependency.get("name") != "ameba_pro2_tools":
                        continue
                    tools_path = (
                        ARDUINO_DATA_DIR
                        / "packages"
                        / dependency.get("packager", "realtek")
                        / "tools"
                        / dependency["name"]
                        / dependency["version"]
                    )
                    if tools_path.is_dir():
                        return tools_path
    raise RuntimeError(
        f"Could not locate the Realtek uploader required by core {core_version}"
    )


def validated_upload_image():
    image_path = ARDUINO_OUTPUT_DIR / "flash_ntz.bin"
    if not BUILD_STAMP.is_file() or not image_path.is_file():
        raise RuntimeError(
            f"No complete firmware artifact exists for {env.subst('$PIOENV')}; build it first"
        )
    try:
        manifest = json.loads(BUILD_STAMP.read_text(encoding="ascii"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Invalid build manifest: {BUILD_STAMP}") from error

    expected = {
        "variant": env.subst("$PIOENV"),
        "core": CORE_SPEC,
        "fqbn": FQBN,
        "fc_link": FC_LINK,
        "ondevice_vision": ONDEVICE_VISION,
    }
    mismatches = [
        f"{key}={manifest.get(key)!r} (expected {value!r})"
        for key, value in expected.items()
        if manifest.get(key) != value
    ]
    digest = sha256_file(image_path)
    if manifest.get("flash_sha256") != digest:
        mismatches.append(
            f"flash_sha256={manifest.get('flash_sha256')!r} (actual {digest!r})"
        )
    if mismatches:
        raise RuntimeError("Refusing mismatched upload artifact: " + "; ".join(mismatches))
    return image_path, digest


def stage_realtek_upload_image(image_path, expected_digest, core_version):
    uploader_dir = realtek_uploader_directory(core_version)
    shared_image = uploader_dir / "flash_ntz.bin"
    temporary_image = uploader_dir / f"flash_ntz.{os.getpid()}.tmp"
    try:
        shutil.copy2(image_path, temporary_image)
        if sha256_file(temporary_image) != expected_digest:
            raise RuntimeError("The staged Realtek upload image failed its hash check")
        os.replace(temporary_image, shared_image)
    finally:
        if temporary_image.exists():
            temporary_image.unlink()
    if sha256_file(shared_image) != expected_digest:
        raise RuntimeError("The Realtek uploader image changed while it was staged")
    return shared_image


def stage_sources():
    if SKETCH_DIR.exists():
        shutil.rmtree(SKETCH_DIR)
    SKETCH_DIR.mkdir(parents=True)
    for source in SRC_DIR.iterdir():
        if source.is_file() and source.suffix.lower() in (".cpp", ".c", ".h", ".hpp"):
            shutil.copy2(source, SKETCH_DIR / source.name)
    (SKETCH_DIR / "BuildConfig.h").write_text(
        "#pragma once\n"
        f"#define BW21CAM_ENABLE_FC_LINK {FC_LINK}\n"
        f"#define BW21CAM_ENABLE_ONDEVICE_VISION {ONDEVICE_VISION}\n"
        f'#define BW21CAM_BUILD_VARIANT "{env.subst("$PIOENV")}"\n',
        encoding="ascii",
    )
    sketch_manifest = (
        "// PlatformIO stages src/ here for the official AmebaPro2 Arduino builder.\n"
    )
    if ONDEVICE_VISION == "1":
        # Realtek's prebuild hook discovers NN models by scanning the .ino file.
        sketch_manifest += (
            "#include \"NNObjectDetection.h\"\n"
            "#include \"VideoStream.h\"\n"
            "static void __attribute__((used)) bw21ModelManifest()\n"
            "{\n"
            "    VideoSetting video(576, 320, 10, VIDEO_RGB, 0);\n"
            "    Camera.configVideoChannel(3, video);\n"
            "    NNObjectDetection manifest;\n"
            "    manifest.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, "
            "NA_MODEL, NA_MODEL);\n"
            "}\n"
        )
    (SKETCH_DIR / "BW21Cam.ino").write_text(sketch_manifest, encoding="ascii")


def build_firmware(target, source, env):
    try:
        ensure_toolchain()
        stage_sources()
        compile_arguments = [
            "compile",
            "--fqbn",
            FQBN,
            "--build-path",
            str(ARDUINO_BUILD_DIR),
            "--warnings",
            "all",
            str(SKETCH_DIR),
        ]
        for attempt in range(1, 4):
            try:
                if ARDUINO_BUILD_DIR.exists():
                    shutil.rmtree(ARDUINO_BUILD_DIR)
                ARDUINO_BUILD_DIR.mkdir(parents=True)
                with realtek_tool_lock():
                    if ONDEVICE_VISION == "1":
                        # The official prebuild hook consumes this temporary tool package.
                        ensure_nn_models()
                    run_cli(compile_arguments)
                break
            except RuntimeError:
                if attempt == 3:
                    raise
                print(f"Realtek compiler failed; retrying ({attempt}/3)...")
                time.sleep(1)
        _, _, core_version = CORE_SPEC.partition("@")
        vision_image = verify_vision_image(
            ARDUINO_BUILD_DIR / "flash_ntz.bin", core_version
        )
        ARDUINO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        for artifact_name in ("flash_ntz.bin", "application.ntz", "application.ntz.map"):
            artifact = ARDUINO_BUILD_DIR / artifact_name
            if not artifact.is_file():
                raise RuntimeError(f"Expected build artifact was not produced: {artifact}")
            shutil.copy2(artifact, ARDUINO_OUTPUT_DIR / artifact_name)
        flash_sha256 = sha256_file(ARDUINO_OUTPUT_DIR / "flash_ntz.bin")
        BUILD_STAMP.write_text(
            json.dumps(
                {
                    "built_at": datetime.now(timezone.utc).isoformat(),
                    "variant": env.subst("$PIOENV"),
                    "core": CORE_SPEC,
                    "fqbn": FQBN,
                    "fc_link": FC_LINK,
                    "ondevice_vision": ONDEVICE_VISION,
                    "flash_sha256": flash_sha256,
                    "vision_image": vision_image,
                },
                indent=2,
            )
            + "\n",
            encoding="ascii",
        )
        print(f"BW21 firmware artifacts: {ARDUINO_OUTPUT_DIR}")
        return 0
    except Exception as error:
        print(f"Error: {error}")
        return 1


def setup_toolchain(target, source, env):
    try:
        ensure_toolchain()
        print(f"AmebaPro2 toolchain ready: {cli_executable()}")
        return 0
    except Exception as error:
        print(f"Error: {error}")
        return 1


def detect_upload_port():
    command_line_port = env.subst("$UPLOAD_PORT").strip()
    if command_line_port and "$" not in command_line_port:
        return command_line_port

    configured = env.GetProjectOption("upload_port", "").strip()
    if configured:
        return configured

    try:
        from serial.tools import list_ports

        ports = [port.device for port in list_ports.comports()]
    except Exception:
        ports = []
    if len(ports) == 1:
        return ports[0]
    if ports:
        raise RuntimeError(
            "Multiple serial ports found. Set upload_port in platformio.ini: "
            + ", ".join(ports)
        )
    raise RuntimeError("No serial port found. Connect the BW21 and set upload_port if needed")


def upload_firmware(target, source, env):
    try:
        ensure_toolchain()
        image_path, image_digest = validated_upload_image()
        _, _, core_version = CORE_SPEC.partition("@")
        port = detect_upload_port()
        with realtek_tool_lock():
            shared_image = stage_realtek_upload_image(
                image_path, image_digest, core_version
            )
            print(
                f"Staged {env.subst('$PIOENV')} upload image: "
                f"sha256={image_digest}"
            )
            upload_output = run_cli(
                [
                    "upload",
                    "--port",
                    port,
                    "--fqbn",
                    FQBN,
                    "--input-file",
                    str(image_path),
                    str(SKETCH_DIR),
                ],
                capture=True,
            )
            if sha256_file(shared_image) != image_digest:
                raise RuntimeError("The Realtek upload image changed during flashing")
        failure_markers = ("upload fail", "uart boot fail", "flashloader loading fail")
        if any(marker in upload_output.lower() for marker in failure_markers):
            raise RuntimeError("Realtek uploader reported a flash failure")
        return 0
    except Exception as error:
        print(f"Error: {error}")
        return 1


source_nodes = [env.File(str(path)) for path in SRC_DIR.iterdir() if path.is_file()]
source_nodes.extend(
    [
        env.File(str(PROJECT_DIR / "platformio.ini")),
        env.File(str(PROJECT_DIR / "platform" / "amebapro2" / "builder" / "main.py")),
    ]
)

firmware = env.Command(
    str(BUILD_STAMP),
    source_nodes,
    env.VerboseAction(build_firmware, "Compiling BW21-CBV firmware"),
)
Default(firmware)

env.AddCustomTarget(
    name="setup",
    dependencies=None,
    actions=env.VerboseAction(setup_toolchain, "Installing AmebaPro2 toolchain"),
    title="Setup AmebaPro2",
    description="Install the pinned Arduino CLI and official Realtek core",
)

env.AddPlatformTarget(
    name="upload",
    dependencies=firmware,
    actions=env.VerboseAction(upload_firmware, "Uploading BW21-CBV firmware"),
    title="Upload",
    description="Upload firmware to the BW21-CBV-Kit over USB serial",
)
