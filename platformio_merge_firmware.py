#!/usr/bin/env python3
"""Merge existing PlatformIO ESP32 build outputs into one 0x0 flash image.

This script reads the existing .pio/build/<env>/idedata.json file and calls
esptool. It does not invoke platformio, so it will not trigger a project build.

Default output name format: [firmware-source]_firmware_<YYYYmmddHHMM>.bin.
"""

from __future__ import annotations

import argparse
import configparser
import json
import re
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Merge existing PlatformIO ESP32 build binaries into one firmware "
            "image flashed at 0x0."
        )
    )
    parser.add_argument(
        "-B",
        "--build-dir",
        type=Path,
        default=None,
        help=(
            "PlatformIO build directory. Defaults to this project's "
            ".pio/build/<default_envs> directory."
        ),
    )
    parser.add_argument(
        "-e",
        "--environment",
        help=(
            "PlatformIO environment name. Defaults to platformio.default_envs "
            "or the build directory name."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help=(
            "Output firmware file. Defaults to "
            "[firmware-source]_firmware_<date>.bin in the project directory."
        ),
    )
    parser.add_argument(
        "--name",
        help="Override the firmware source name used in the output file.",
    )
    parser.add_argument(
        "--chip",
        help="Override chip name. Defaults to the PlatformIO board definition.",
    )
    parser.add_argument(
        "--flash-mode",
        help="Override flash mode. Defaults to the PlatformIO board definition.",
    )
    parser.add_argument(
        "--flash-freq",
        help="Override flash frequency. Defaults to the PlatformIO board definition.",
    )
    parser.add_argument(
        "--flash-size",
        help="Override flash size. Defaults to PlatformIO upload flash_size.",
    )
    parser.add_argument(
        "--flash-offset",
        default="0x0",
        help="Base flash offset for the merged raw image. Defaults to 0x0.",
    )
    parser.add_argument(
        "--fill-flash-size",
        help="Pad the output to this flash size. Defaults to --flash-size.",
    )
    parser.add_argument(
        "--no-fill",
        action="store_true",
        help="Do not pad the output image to the configured flash size.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the esptool command without running it.",
    )
    return parser.parse_args()


def load_json(path: Path, *, required: bool = True) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError:
        if not required:
            return {}
        raise SystemExit(
            f"Missing {path}. Build the PlatformIO environment once before "
            "merging firmware."
        )
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON in {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise SystemExit(f"Invalid {path}: expected a JSON object.")
    return data


def load_platformio_config() -> configparser.ConfigParser:
    config = configparser.ConfigParser(interpolation=None)
    config.optionxform = str
    config.read(PROJECT_DIR / "platformio.ini", encoding="utf-8")
    return config


def resolve_path(path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    return (Path.cwd() / path).resolve()


def safe_name(name: str) -> str:
    name = name.strip().strip("[]")
    name = re.sub(r'[<>:"/\\|?*\x00-\x1F]+', "_", name)
    name = re.sub(r"\s+", "_", name)
    name = re.sub(r"_+", "_", name)
    name = name.strip("._ ")
    return name or "firmware"


def default_environment(config: configparser.ConfigParser) -> str:
    if config.has_option("platformio", "default_envs"):
        default_envs = config.get("platformio", "default_envs").strip()
        if default_envs:
            return default_envs.splitlines()[0].strip()
    return "original_test"


def environment_options(
    config: configparser.ConfigParser, environment: str
) -> dict[str, str]:
    options: dict[str, str] = {}
    for section in ("env", f"env:{environment}"):
        if config.has_section(section):
            options.update(dict(config.items(section)))
    return options


def board_file(config: configparser.ConfigParser, board: str) -> Path | None:
    boards_dir = PROJECT_DIR / "boards"
    if config.has_option("platformio", "boards_dir"):
        configured = Path(config.get("platformio", "boards_dir").strip())
        boards_dir = configured if configured.is_absolute() else PROJECT_DIR / configured

    candidate = boards_dir / f"{board}.json"
    if candidate.is_file():
        return candidate
    return None


def normalize_flash_freq(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip().strip('"').strip("'")
    if not text:
        return None
    if text.lower().endswith("l"):
        text = text[:-1]
    if text.isdigit():
        hz = int(text)
        if hz % 1_000_000 == 0:
            return f"{hz // 1_000_000}m"
        if hz % 1_000 == 0:
            return f"{hz // 1_000}k"
    return text


def image_header_flash_settings(path: Path) -> dict[str, str]:
    mode_names = {
        0x00: "qio",
        0x01: "qout",
        0x02: "dio",
        0x03: "dout",
    }
    size_names = {
        0x00: "1MB",
        0x01: "2MB",
        0x02: "4MB",
        0x03: "8MB",
        0x04: "16MB",
        0x05: "32MB",
        0x06: "64MB",
        0x07: "128MB",
    }
    freq_names = {
        0x00: "40m",
        0x01: "26m",
        0x02: "20m",
        0x0F: "80m",
    }

    try:
        header = path.read_bytes()[:4]
    except OSError:
        return {}
    if len(header) < 4 or header[0] != 0xE9:
        return {}

    size_freq = header[3]
    settings: dict[str, str] = {}
    mode = mode_names.get(header[2])
    size = size_names.get(size_freq >> 4)
    freq = freq_names.get(size_freq & 0x0F)
    if mode:
        settings["flash_mode"] = mode
    if size:
        settings["flash_size"] = size
    if freq:
        settings["flash_freq"] = freq
    return settings


def strip_inline_comment(value: str) -> str:
    return value.split(";", 1)[0].strip()


def project_name(environment: str, override: str | None) -> str:
    return safe_name(override or environment)


def default_output_name(name: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d%H%M")
    return f"[{name}]_firmware_{timestamp}.bin"


def format_command(command: list[str]) -> str:
    if sys.platform == "win32":
        return subprocess.list2cmdline(command)
    return " ".join(shlex.quote(part) for part in command)


def esptool_supports_modern_merge() -> bool:
    try:
        result = subprocess.run(
            [sys.executable, "-m", "esptool", "merge-bin", "--help"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except FileNotFoundError:
        return False
    return result.returncode == 0


def flash_files_from_idedata(build_dir: Path, idedata: dict[str, Any]) -> list[tuple[str, Path]]:
    extra = idedata.get("extra", {})
    if not isinstance(extra, dict):
        extra = {}

    files: list[tuple[str, Path]] = []
    flash_images = extra.get("flash_images", [])
    if isinstance(flash_images, list):
        for image in flash_images:
            if not isinstance(image, dict):
                continue
            offset = image.get("offset")
            path = image.get("path")
            if offset is None or path is None:
                continue
            files.append((str(offset), resolve_build_path(build_dir, Path(str(path)))))

    app_offset = extra.get("application_offset", "0x10000")
    app_path = build_dir / "firmware.bin"
    files.append((str(app_offset), app_path.resolve()))

    if not files:
        files = [
            ("0x0000", (build_dir / "bootloader.bin").resolve()),
            ("0x8000", (build_dir / "partitions.bin").resolve()),
            ("0x10000", app_path.resolve()),
        ]

    return sorted(files, key=lambda item: int(item[0], 0))


def resolve_build_path(build_dir: Path, path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    return (build_dir / path).resolve()


def validate_build_files(flash_files: list[tuple[str, Path]]) -> None:
    missing_files: list[str] = []
    for offset, file_path in flash_files:
        if not file_path.is_file():
            missing_files.append(f"  {offset}: {file_path}")

    if missing_files:
        missing = "\n".join(missing_files)
        raise SystemExit(
            f"Missing build output files:\n{missing}\nBuild the PlatformIO "
            "environment once before merging firmware."
        )


def append_flash_settings(
    command: list[str],
    *,
    flash_mode: str | None,
    flash_freq: str | None,
    flash_size: str | None,
    modern: bool,
) -> None:
    settings = [
        ("--flash-mode" if modern else "--flash_mode", flash_mode),
        ("--flash-freq" if modern else "--flash_freq", flash_freq),
        ("--flash-size" if modern else "--flash_size", flash_size),
    ]
    for option, value in settings:
        if value:
            command.extend([option, str(value)])


def build_esptool_command(
    *,
    chip: str,
    output: Path,
    flash_offset: str,
    fill_flash_size: str | None,
    flash_mode: str | None,
    flash_freq: str | None,
    flash_size: str | None,
    flash_files: list[tuple[str, Path]],
    modern: bool,
) -> list[str]:
    command = [sys.executable, "-m", "esptool", "--chip", chip]
    if modern:
        command.extend(
            [
                "merge-bin",
                "--output",
                str(output),
                "--format",
                "raw",
                "--target-offset",
                flash_offset,
            ]
        )
        if fill_flash_size:
            command.extend(["--pad-to-size", str(fill_flash_size)])
    else:
        command.extend(
            [
                "merge_bin",
                "-o",
                str(output),
                "-f",
                "raw",
                "-t",
                flash_offset,
            ]
        )
        if fill_flash_size:
            command.extend(["--fill-flash-size", str(fill_flash_size)])

    append_flash_settings(
        command,
        flash_mode=flash_mode,
        flash_freq=flash_freq,
        flash_size=flash_size,
        modern=modern,
    )
    for offset, file_path in flash_files:
        command.extend([offset, str(file_path)])
    return command


def main() -> int:
    args = parse_args()
    config = load_platformio_config()

    environment = args.environment or default_environment(config)
    build_dir = (
        resolve_path(args.build_dir)
        if args.build_dir is not None
        else (PROJECT_DIR / ".pio" / "build" / environment).resolve()
    )
    if args.environment is None:
        environment = build_dir.name

    idedata = load_json(build_dir / "idedata.json")
    env_options = environment_options(config, environment)

    board_name = strip_inline_comment(env_options.get("board", ""))
    board_data: dict[str, Any] = {}
    if board_name:
        board_path = board_file(config, board_name)
        if board_path is not None:
            board_data = load_json(board_path, required=False)

    build_data = board_data.get("build", {})
    if not isinstance(build_data, dict):
        build_data = {}
    upload_data = board_data.get("upload", {})
    if not isinstance(upload_data, dict):
        upload_data = {}

    chip = args.chip or build_data.get("mcu")
    if not chip:
        raise SystemExit(
            "Unable to determine chip. Pass --chip, for example: --chip esp32s3"
        )
    chip = str(chip)

    flash_files = flash_files_from_idedata(build_dir, idedata)
    validate_build_files(flash_files)
    bootloader_path = next(
        (path for offset, path in flash_files if int(offset, 0) == 0), None
    )
    header_settings = (
        image_header_flash_settings(bootloader_path) if bootloader_path else {}
    )

    flash_mode = (
        args.flash_mode
        or header_settings.get("flash_mode")
        or build_data.get("flash_mode")
    )
    flash_freq = (
        args.flash_freq
        or header_settings.get("flash_freq")
        or normalize_flash_freq(build_data.get("f_flash"))
    )
    flash_size = (
        args.flash_size
        or header_settings.get("flash_size")
        or strip_inline_comment(env_options.get("board_upload.flash_size", ""))
        or upload_data.get("flash_size")
    )
    flash_size = str(flash_size) if flash_size else None

    name = project_name(environment, args.name)
    output = (
        PROJECT_DIR / default_output_name(name)
        if args.output is None
        else resolve_path(args.output)
    )
    fill_flash_size = None if args.no_fill else (args.fill_flash_size or flash_size)

    modern_esptool = esptool_supports_modern_merge()
    command = build_esptool_command(
        chip=chip,
        output=output,
        flash_offset=args.flash_offset,
        fill_flash_size=fill_flash_size,
        flash_mode=flash_mode,
        flash_freq=flash_freq,
        flash_size=flash_size,
        flash_files=flash_files,
        modern=modern_esptool,
    )

    print(f"Build dir: {build_dir}")
    print(f"Environment: {environment}")
    print(f"Project:   {name}")
    print(f"Output:    {output}")
    print(f"Chip:      {chip}")
    print(f"Esptool:   {'modern' if modern_esptool else 'legacy'}")
    print(f"Flash:     mode={flash_mode or 'default'} freq={flash_freq or 'default'} size={flash_size or 'default'}")
    if fill_flash_size:
        print(f"Fill size: {fill_flash_size}")
    else:
        print("Fill size: disabled")
    print()
    print("Flash files:")
    for offset, file_path in flash_files:
        print(f"  {offset}: {file_path}")
    print()
    print("Command:")
    print(f"  {format_command(command)}")

    if args.dry_run:
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    sys.stdout.flush()
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError as exc:
        raise SystemExit(f"Failed to run Python: {exc}") from exc
    except subprocess.CalledProcessError as exc:
        return exc.returncode

    print()
    print(f"Merged firmware created: {output}")
    print(f"Flash this file at offset: {args.flash_offset}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
