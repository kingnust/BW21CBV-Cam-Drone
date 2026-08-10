import hashlib
import json
import os
import platform
import shutil
import subprocess
import tarfile
import urllib.request
import zipfile
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
ARDUINO_DATA_DIR = TOOLS_DIR / "arduino-data"
ARDUINO_DOWNLOADS_DIR = TOOLS_DIR / "arduino-downloads"
ARDUINO_USER_DIR = TOOLS_DIR / "arduino-user"
ARDUINO_CONFIG = TOOLS_DIR / "arduino-cli.yaml"
SKETCH_DIR = WORK_DIR / "sketch" / "BW21Cam"
ARDUINO_BUILD_DIR = WORK_DIR / "arduino-build"
ARDUINO_OUTPUT_DIR = BUILD_DIR / "firmware"
BUILD_STAMP = BUILD_DIR / "bw21-build.json"

CLI_VERSION = env.GetProjectOption("custom_arduino_cli_version")
CORE_SPEC = env.GetProjectOption("custom_amebapro2_core")
PACKAGE_INDEX = env.GetProjectOption("custom_amebapro2_index")
FQBN = env.GetProjectOption("custom_amebapro2_fqbn")
FC_LINK = env.GetProjectOption("custom_fc_link", "0")


def cli_executable():
    executable = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    return TOOLS_DIR / "arduino-cli" / executable


def write_cli_config():
    for directory in (
        TOOLS_DIR,
        ARDUINO_DATA_DIR,
        ARDUINO_DOWNLOADS_DIR,
        ARDUINO_USER_DIR,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    def yaml_path(path):
        return path.resolve().as_posix()

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


def stage_sources():
    if SKETCH_DIR.exists():
        shutil.rmtree(SKETCH_DIR)
    SKETCH_DIR.mkdir(parents=True)
    for source in SRC_DIR.iterdir():
        if source.is_file() and source.suffix.lower() in (".cpp", ".c", ".h", ".hpp"):
            shutil.copy2(source, SKETCH_DIR / source.name)
    (SKETCH_DIR / "BuildConfig.h").write_text(
        "#pragma once\n"
        f"#define BW21CAM_ENABLE_FC_LINK {FC_LINK}\n",
        encoding="ascii",
    )
    (SKETCH_DIR / "BW21Cam.ino").write_text(
        "// PlatformIO stages src/ here for the official AmebaPro2 Arduino builder.\n",
        encoding="ascii",
    )


def build_firmware(target, source, env):
    try:
        ensure_toolchain()
        stage_sources()
        ARDUINO_BUILD_DIR.mkdir(parents=True, exist_ok=True)
        run_cli(
            [
                "compile",
                "--fqbn",
                FQBN,
                "--build-path",
                str(ARDUINO_BUILD_DIR),
                "--warnings",
                "all",
                str(SKETCH_DIR),
            ]
        )
        if ARDUINO_OUTPUT_DIR.exists():
            shutil.rmtree(ARDUINO_OUTPUT_DIR)
        ARDUINO_OUTPUT_DIR.mkdir(parents=True)
        for artifact_name in ("flash_ntz.bin", "application.ntz", "application.ntz.map"):
            artifact = ARDUINO_BUILD_DIR / artifact_name
            if not artifact.is_file():
                raise RuntimeError(f"Expected build artifact was not produced: {artifact}")
            shutil.copy2(artifact, ARDUINO_OUTPUT_DIR / artifact_name)
        BUILD_STAMP.write_text(
            json.dumps(
                {
                    "built_at": datetime.now(timezone.utc).isoformat(),
                    "core": CORE_SPEC,
                    "fqbn": FQBN,
                    "fc_link": FC_LINK,
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
        port = detect_upload_port()
        run_cli(
            [
                "upload",
                "--port",
                port,
                "--fqbn",
                FQBN,
                "--input-dir",
                str(ARDUINO_BUILD_DIR),
                str(SKETCH_DIR),
            ]
        )
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
