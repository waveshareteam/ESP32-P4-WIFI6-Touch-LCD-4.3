#!/usr/bin/env python3
"""Run fast, dependency-free repository checks used by every Actions route."""

from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


REPO_ROOT = Path(__file__).resolve().parents[2]
EXAMPLES_ROOT = REPO_ROOT / "examples" / "esp-idf"
LINK_PATTERN = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
REMOTE_SCHEMES = {"http", "https", "mailto", "tel", "data"}
EXCLUDED_DIRECTORY_NAMES = {".git", "build", "managed_components", "__pycache__"}
REQUIRED_PAIRS = (
    ("README.md", "README_ZH.md"),
    ("CONTRIBUTING.md", "CONTRIBUTING_ZH.md"),
    ("SUPPORT.md", "SUPPORT_ZH.md"),
    ("docs/README.md", "docs/README_ZH.md"),
    ("docs/CI.md", "docs/CI_ZH.md"),
    ("docs/CI_FIRMWARE.md", "docs/CI_FIRMWARE_ZH.md"),
    ("docs/COMPONENTS.md", "docs/COMPONENTS_ZH.md"),
    ("docs/HARDWARE.md", "docs/HARDWARE_ZH.md"),
    ("firmware/brookesia/README.md", "firmware/brookesia/README_ZH.md"),
    ("examples/arduino/README.md", "examples/arduino/README_ZH.md"),
    (
        "examples/esp-idf/09_video_lcd_display/README.md",
        "examples/esp-idf/09_video_lcd_display/README_ZH.md",
    ),
    (
        "examples/esp-idf/10_mp4_player/README.md",
        "examples/esp-idf/10_mp4_player/README_ZH.md",
    ),
    (
        "examples/esp-idf/11_esp_brookesia_phone/README.md",
        "examples/esp-idf/11_esp_brookesia_phone/README_ZH.md",
    ),
)
REQUIRED_CI_DEFAULTS = (
    "config/ci/rev3_x.defaults",
    "config/ci/rev1_3.defaults",
    "config/ci/i2s_echo.defaults",
    "config/ci/rgb888.defaults",
    "config/ci/usb_rgb888.defaults",
    "config/ci/brookesia_ai.defaults",
    "config/ci/usb_minimal.defaults",
)
REQUIRED_CI_HELPERS = (
    ".github/workflows/esp-idf-examples.yml",
    ".github/workflows/arduino-examples.yml",
    ".github/scripts/audit_markdown.py",
    ".github/scripts/discover_esp_idf_examples.py",
    ".github/scripts/discover_arduino_examples.py",
    ".github/tests/run_tests.py",
    ".github/tests/test_discover_esp_idf_examples.py",
    ".github/tests/test_discover_arduino_examples.py",
)
ARDUINO_LIBRARY_REQUIREMENTS = {
    "GFX_Library_for_Arduino": {
        "version": "1.6.0",
        "files": (
            "README.md",
            "license.txt",
            "src/Arduino_GFX_Library.h",
            "src/databus/Arduino_ESP32DSIPanel.cpp",
        ),
    },
    "lvgl": {
        "version": "9.3.0",
        "files": (
            "COPYRIGHTS.md",
            "LICENCE.txt",
            "README.md",
            "src/core/lv_obj.c",
            "src/demos/benchmark/screenshot1.png",
            "src/demos/keypad_encoder/screenshot1.png",
            "src/demos/music/screenshot1.gif",
            "src/demos/stress/screenshot1.png",
        ),
    },
}
ARDUINO_LV_CONF_MARKERS = (
    "#define LV_COLOR_DEPTH 16",
    "#define LV_USE_OS   LV_OS_NONE",
    "#define LV_USE_DEMO_WIDGETS 1",
)
REQUIRED_HOMEPAGE_COMPONENTS = {
    "centered_header",
    "html_h1",
    "subtitle",
    "badges",
    "language_switch",
    "quick_links",
    "hero_image",
    "separator",
    "h2",
}
REQUIRED_HOMEPAGE_QUICK_LINKS = {"product", "documentation", "firmware", "esp_idf"}
REQUIRED_HOMEPAGE_BADGES = {"build", "license"}
REQUIRED_HOMEPAGE_H2_ICONS = ["✨", "🖥️", "📦", "🧪", "🛠️", "🗂️", "📚", "🤝", "📄"]
BSP_COMPONENT = "waveshare/esp32_p4_wifi6_touch_lcd_4_3"
BSP_PIN_FIELDS = {
    "version": "==1.0.1",
}
BSP_BASE_COMPONENT_DIRS = (
    "examples/esp-idf/06_I2SCodec/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/07_Displaycolorbar/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/08_lvgl_demo_v9/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/09_video_lcd_display/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/10_mp4_player/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/11_esp_brookesia_phone/components/esp32_p4_wifi6_touch_lcd_4_3",
    "examples/esp-idf/12_usb_extend_screen/components/esp32_p4_wifi6_touch_lcd_4_3",
)
BSP_EXTRA_COMPONENT_DIRS = (
    "examples/esp-idf/12_usb_extend_screen/components/bsp_extra",
)
BSP_FORBIDDEN_EXTENSION_DIRS = (
    "examples/esp-idf/08_lvgl_demo_v9/components/bsp_extra",
)
BSP_MANIFESTS = (
    "examples/esp-idf/06_I2SCodec/main/idf_component.yml",
    "examples/esp-idf/07_Displaycolorbar/main/idf_component.yml",
    "examples/esp-idf/08_lvgl_demo_v9/main/idf_component.yml",
    "examples/esp-idf/09_video_lcd_display/main/idf_component.yml",
    "examples/esp-idf/10_mp4_player/main/idf_component.yml",
    "examples/esp-idf/11_esp_brookesia_phone/main/idf_component.yml",
    "examples/esp-idf/12_usb_extend_screen/components/bsp_extra/idf_component.yml",
)
FACTORY_FIRMWARE_IMAGES = {
    "firmware/ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260206.bin": (
        33_488_896,
        "f87b4b16f49704dc8b05b44953a45c011ca9c244e05547e035b4bfa3db74e022",
    ),
    "firmware/ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260820.bin": (
        16_777_216,
        "60f1ea3c77d75c95bd1de75b12642fe11c53873e0290a9b96bb6bb7a4c3aad78",
    ),
    "firmware/ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-261106.bin": (
        16_777_216,
        "e8cdef6903dc944680fbb3cbdd2165815736be0b11a8129232c1b4e177afd45b",
    ),
}


def direct_examples(repo_root: Path = REPO_ROOT) -> list[Path]:
    root = repo_root / "examples" / "esp-idf"
    if not root.is_dir():
        return []
    return sorted(
        path
        for path in root.iterdir()
        if path.is_dir() and (path / "CMakeLists.txt").is_file() and (path / "main").is_dir()
    )


def factory_firmware_integrity_errors(
    repo_root: Path,
    *,
    expected_images: dict[str, tuple[int, str]] | None = None,
) -> list[str]:
    """Require every reviewed factory image identity without modifying artifacts."""
    if expected_images is None:
        expected_images = FACTORY_FIRMWARE_IMAGES
    firmware_root = repo_root / "firmware"
    binaries = sorted(
        path.relative_to(repo_root).as_posix()
        for path in firmware_root.rglob("*")
        if path.is_file()
        and path.suffix.lower() == ".bin"
        and not any(
            part in EXCLUDED_DIRECTORY_NAMES or part.startswith("build-")
            for part in path.relative_to(firmware_root).parts[:-1]
        )
    ) if firmware_root.is_dir() else []
    errors: list[str] = []
    expected_paths = sorted(expected_images)
    if binaries != expected_paths:
        errors.append(
            f"expected immutable factory firmware binaries at {', '.join(expected_paths)}; "
            f"found {', '.join(binaries) if binaries else 'none'}"
        )

    for expected_path, (expected_size, expected_sha256) in expected_images.items():
        image = repo_root / expected_path
        if not image.is_file():
            continue
        size = image.stat().st_size
        if size != expected_size:
            errors.append(
                f"{expected_path}: immutable size changed (expected {expected_size}, found {size})"
            )
        hasher = hashlib.sha256()
        with image.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                hasher.update(chunk)
        digest = hasher.hexdigest()
        if digest != expected_sha256:
            errors.append(f"{expected_path}: immutable SHA-256 changed")
    return errors


def pinned_bsp_dependency_errors(manifest: str) -> list[str]:
    """Require the exact published BSP release without source overrides."""
    key_pattern = re.compile(rf"(?m)^  {re.escape(BSP_COMPONENT)}:[ \t]*$")
    matches = list(key_pattern.finditer(manifest))
    if len(matches) != 1:
        return [f"expected exactly one {BSP_COMPONENT} dependency mapping"]

    fields: dict[str, str] = {}
    errors: list[str] = []
    for line in manifest[matches[0].end() :].lstrip("\r\n").splitlines():
        if not line.startswith("    "):
            break
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if ":" not in stripped:
            errors.append("BSP dependency mapping contains an invalid field")
            continue
        field, value = stripped.split(":", 1)
        field = field.strip()
        if field in fields:
            errors.append(f"BSP dependency mapping repeats {field}")
        fields[field] = value.strip().strip("\"'")

    if fields != BSP_PIN_FIELDS:
        errors.append(
            "BSP dependency must use the exact published Registry version without git/path fields"
        )
    return errors


def bsp_pin_policy_errors(repo_root: Path = REPO_ROOT) -> list[str]:
    """Keep local base BSP copies removed while retaining the required product extension."""
    errors: list[str] = []
    for relative in BSP_BASE_COMPONENT_DIRS:
        if (repo_root / relative).exists():
            errors.append(f"{relative}: local base BSP path must be absent")
    for relative in BSP_EXTRA_COMPONENT_DIRS:
        if not (repo_root / relative).is_dir():
            errors.append(f"{relative}: required product bsp_extra tree is missing")
    for relative in BSP_FORBIDDEN_EXTENSION_DIRS:
        path = repo_root / relative
        if path.is_file() or (path.is_dir() and any(child.is_file() for child in path.rglob("*"))):
            errors.append(f"{relative}: unused product extension must remain removed")
    for relative in BSP_MANIFESTS:
        path = repo_root / relative
        try:
            manifest = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{relative}: cannot read BSP manifest: {error}")
            continue
        errors.extend(f"{relative}: {error}" for error in pinned_bsp_dependency_errors(manifest))
    return errors


def flash_size_selection_errors(defaults: str, relative: str) -> list[str]:
    """Require one unambiguous 32 MB product flash selection."""
    selections = re.findall(
        r"(?m)^CONFIG_ESPTOOLPY_FLASHSIZE_([A-Za-z0-9_]+)=y[ \t]*$", defaults
    )
    if selections != ["32MB"]:
        rendered = ", ".join(selections) if selections else "none"
        return [f"{relative}: expected only the 32MB flash selection; found {rendered}"]
    return []


def duplicate_sdkconfig_assignment_errors(defaults: str, relative: str) -> list[str]:
    """Reject repeated active sdkconfig defaults whose ordering is easy to misread."""
    assignments = re.findall(r"(?m)^(CONFIG_[A-Za-z0-9_]+)=", defaults)
    duplicates = sorted({key for key in assignments if assignments.count(key) > 1})
    return [f"{relative}: repeated sdkconfig assignment {key}" for key in duplicates]


def sdkconfig_assignments(defaults: str) -> dict[str, str]:
    """Return the final values from one or more SDKCONFIG_DEFAULTS files.

    ESP-IDF accepts both active assignments and the standard ``# CONFIG_X is
    not set`` form. Keeping the latter matters here because the explicit
    Rev1.3 overlay must override the repository's Rev3.x defaults without
    leaving either hardware-revision choice active.
    """
    assignments: dict[str, str] = {}
    for line in defaults.splitlines():
        active = re.fullmatch(r"(CONFIG_[A-Za-z0-9_]+)=(.+)", line)
        if active:
            assignments[active.group(1)] = active.group(2)
            continue
        disabled = re.fullmatch(r"# (CONFIG_[A-Za-z0-9_]+) is not set", line)
        if disabled:
            assignments[disabled.group(1)] = "n"
    return assignments


def revision_profile_errors(repo_root: Path, examples: list[Path]) -> list[str]:
    """Require conflict-free Rev3.x defaults and the runnable Rev1.3 overlay."""
    profiles = {
        "rev3_x": {
            "CONFIG_ESP32P4_SELECTS_REV_LESS_V3": "n",
            "CONFIG_ESP32P4_REV_MIN_100": "n",
            "CONFIG_ESP32P4_REV_MIN_300": "y",
            "CONFIG_SPIRAM": "y",
            "CONFIG_SPIRAM_SPEED_200M": "n",
            "CONFIG_SPIRAM_SPEED_250M": "y",
            "CONFIG_BOOTLOADER_LOG_LEVEL_WARN": "y",
            "CONFIG_BOOTLOADER_LOG_LEVEL": "2",
        },
        "rev1_3": {
            "CONFIG_ESP32P4_SELECTS_REV_LESS_V3": "y",
            "CONFIG_ESP32P4_REV_MIN_100": "y",
            "CONFIG_ESP32P4_REV_MIN_300": "n",
            "CONFIG_SPIRAM": "y",
            "CONFIG_SPIRAM_SPEED_200M": "y",
            "CONFIG_SPIRAM_SPEED_250M": "n",
            "CONFIG_BOOTLOADER_LOG_LEVEL_WARN": "y",
            "CONFIG_BOOTLOADER_LOG_LEVEL": "2",
        },
    }
    errors: list[str] = []
    overlays: dict[str, str] = {}
    for name in profiles:
        path = repo_root / "config" / "ci" / f"{name}.defaults"
        try:
            overlays[name] = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{path.relative_to(repo_root)}: cannot read revision profile: {error}")
    if errors:
        return errors

    for example in examples:
        defaults = example / "sdkconfig.defaults"
        base = defaults.read_text(encoding="utf-8")
        relative = defaults.relative_to(repo_root).as_posix()
        for name, expected in profiles.items():
            combined = sdkconfig_assignments(f"{base}\n{overlays[name]}")
            for key, value in expected.items():
                if combined.get(key) != value:
                    errors.append(
                        f"{relative}: {name} profile must resolve {key}={value}"
                    )
    return errors


def strip_c_comments_and_literals(source: str) -> str:
    """Blank C/C++ comments and quoted literals while preserving line boundaries."""
    token = re.compile(
        r'//[^\r\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        flags=re.DOTALL,
    )
    return token.sub(lambda match: re.sub(r"[^\r\n]", " ", match.group(0)), source)


def wifi_public_log_errors(source: str) -> list[str]:
    """Prevent the configured Wi-Fi password from entering public serial logs."""
    active_source = strip_c_comments_and_literals(source)
    log_calls = re.findall(
        r"(?:ESP_LOG[A-Z]+|printf|fprintf|puts)\s*\([^;]*\);",
        active_source,
        flags=re.DOTALL,
    )
    if any("EXAMPLE_ESP_WIFI_PASS" in call or ".password" in call for call in log_calls):
        return ["04_wifistation: Wi-Fi credentials must not be written to serial logs"]
    return []


def audio_bsp_contract_errors(source: str, overlay: str, config: str) -> list[str]:
    """Keep example 06 on the public board speaker/microphone codec APIs."""
    active_source = strip_c_comments_and_literals(source)
    required = (
        ("bsp_audio_codec_speaker_init()", r"\bbsp_audio_codec_speaker_init\s*\(\s*\)"),
        ("bsp_audio_codec_microphone_init()", r"\bbsp_audio_codec_microphone_init\s*\(\s*\)"),
        ("esp_codec_dev_read(", r"\besp_codec_dev_read\s*\("),
        ("esp_codec_dev_write(", r"\besp_codec_dev_write\s*\("),
        (".channel = 2", r"\.channel\s*=\s*2\b"),
        (".channel_mask = 0x03", r"\.channel_mask\s*=\s*0x0*3\b"),
        ("CONFIG_EXAMPLE_MODE_ECHO", r"\bCONFIG_EXAMPLE_MODE_ECHO\b"),
    )
    errors = [
        f"06_I2SCodec: missing board-codec path {label}"
        for label, pattern in required
        if re.search(pattern, active_source) is None
    ]
    for label, pattern in (
        ("BSP_I2S_GPIO_CFG", r"\bBSP_I2S_GPIO_CFG\b"),
        ("bsp_audio_poweramp_enable", r"\bbsp_audio_poweramp_enable\s*\("),
        ("i2s_channel_read(", r"\bi2s_channel_read\s*\("),
        ("i2s_channel_write(", r"\bi2s_channel_write\s*\("),
    ):
        if re.search(pattern, active_source):
            errors.append(f"06_I2SCodec: obsolete raw/BSP-private audio path {label}")
    if re.search(r"(?m)^CONFIG_EXAMPLE_MODE_ECHO=y[ \t]*$", overlay) is None:
        errors.append("06_I2SCodec: echo CI overlay must enable the microphone path")
    if re.search(r"(?m)^#define\s+EXAMPLE_MCLK_MULTIPLE\s+\(256\)[ \t]*$", config) is None:
        errors.append("06_I2SCodec: BSP codec and I2S must use the same 256x MCLK contract")
    return errors


def markdown_targets(text: str) -> list[str]:
    targets: list[str] = []
    in_fence = False
    for line in text.splitlines():
        if line.lstrip().startswith(("```", "~~~")):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for match in LINK_PATTERN.finditer(line):
            target = match.group(1).strip()
            if target.startswith("<") and ">" in target:
                target = target[1 : target.index(">")]
            else:
                target = target.split(maxsplit=1)[0]
            targets.append(target)
    return targets


def markdown_files(repo_root: Path = REPO_ROOT) -> list[Path]:
    return sorted(
        path
        for path in repo_root.rglob("*.md")
        if not EXCLUDED_DIRECTORY_NAMES.intersection(path.relative_to(repo_root).parts)
    )


def local_link_errors(repo_root: Path = REPO_ROOT) -> list[str]:
    errors: list[str] = []
    for markdown in markdown_files(repo_root):
        text = markdown.read_text(encoding="utf-8")
        for raw_target in markdown_targets(text):
            if not raw_target or raw_target.startswith("#"):
                continue
            parsed = urlsplit(raw_target)
            if parsed.scheme.lower() in REMOTE_SCHEMES or parsed.netloc:
                continue
            target = unquote(parsed.path).replace("\\", "/")
            if not target or "${" in target or "{{" in target:
                continue
            if target.startswith("/"):
                resolved = repo_root / target.lstrip("/")
            else:
                resolved = markdown.parent / target
            if not resolved.exists():
                relative_markdown = markdown.relative_to(repo_root).as_posix()
                errors.append(f"{relative_markdown}: missing local link target {raw_target!r}")
    return errors


def usb_audio_dependency_errors(
    manifest: str, main_cmake: str, project_cmake: str, tinyusb_config: str
) -> list[str]:
    """Keep the optional UAC package outside the USB minimal build graph."""
    errors: list[str] = []
    required_manifest = (
        'espressif/usb_device_uac:\n'
        '    version: "1.2.0"\n'
        '    # Download for the opt-in UAC path; do not require it for minimal builds.\n'
        '    require: no'
    )
    if required_manifest not in manifest:
        errors.append("12_usb_extend_screen: usb_device_uac must stay optional in the manifest")

    required_cmake = (
        "if(CONFIG_UAC_AUDIO_ENABLE)\n"
        '    list (APPEND srcs "app_uac.c")\n'
        "    list(APPEND priv_requires espressif__usb_device_uac)\n"
        "endif()"
    )
    if required_cmake not in main_cmake or "PRIV_REQUIRES ${priv_requires}" not in main_cmake:
        errors.append("12_usb_extend_screen: UAC component link must remain conditional")

    required_private_definition = (
        "if(NOT CONFIG_UAC_AUDIO_ENABLE)\n"
        "    # project() creates managed component targets; an older IDF can still compile\n"
        "    # this downloaded-but-unlinked UAC target, so only it needs TinyUSB audio declarations.\n"
        "    idf_component_get_property(uac_lib espressif__usb_device_uac COMPONENT_LIB)\n"
        "    if(TARGET ${uac_lib})\n"
        "        target_compile_definitions(${uac_lib} PRIVATE CFG_TUD_AUDIO=1)\n"
        "    endif()\n"
        "endif()"
    )
    project_call = "project(usb_touch_screen)"
    project_index = project_cmake.find(project_call)
    definition_index = project_cmake.find(required_private_definition)
    if definition_index == -1:
        errors.append(
            "12_usb_extend_screen: disabled UAC needs a post-project, TARGET-guarded, target-private audio declaration"
        )
    elif project_index == -1 or definition_index < project_index:
        errors.append(
            "12_usb_extend_screen: disabled UAC audio declaration must follow top-level project()"
        )
    if "target_compile_definitions(${uac_lib} PRIVATE CFG_TUD_AUDIO=1)" in main_cmake:
        errors.append(
            "12_usb_extend_screen: disabled UAC audio declaration must not run from main/CMakeLists.txt"
        )

    required_audio_guard = (
        "#ifndef CFG_TUD_AUDIO\n"
        "#if CONFIG_UAC_AUDIO_ENABLE\n"
        "#define CFG_TUD_AUDIO             1\n"
        "#else\n"
        "#define CFG_TUD_AUDIO             0\n"
        "#endif\n"
        "#endif"
    )
    if required_audio_guard not in tinyusb_config:
        errors.append("12_usb_extend_screen: TinyUSB audio setting must allow a private override")
    return errors


def brookesia_ai_compatibility_errors(cmake: str, coze_source: str) -> list[str]:
    """Keep the GMF 0.6 C++ compatibility layer local to the optional AI path."""
    errors: list[str] = []
    required_cxx_flag = (
        'set(SRCS_CPP_COMPILE_FLAGS "-Wno-missing-field-initializers -Wno-format")\n'
        "if(CONFIG_ESP_BROOKESIA_ENABLE_AI_FRAMEWORK)\n"
        '    string(APPEND SRCS_CPP_COMPILE_FLAGS " -fpermissive")\n'
        "endif()"
    )
    if required_cxx_flag not in cmake:
        errors.append(
            "11_esp_brookesia_phone: AI profile requires a local C++ -fpermissive compatibility flag"
        )
    if "esp_gmf_afe_keep_awake" in coze_source:
        errors.append(
            "11_esp_brookesia_phone: GMF 0.6 source must not call esp_gmf_afe_keep_awake"
        )
    return errors


def homepage_policy_errors(audit_policy: object) -> list[str]:
    """Require the configured bilingual homepage to retain its single-product contract."""
    if not isinstance(audit_policy, dict):
        return ["Markdown audit policy must be a JSON object"]

    homepage_pairs = audit_policy.get("homepage_pairs")
    if not isinstance(homepage_pairs, list):
        return ["Markdown audit policy: missing homepage_pairs list for README.md and README_ZH.md"]

    homepage = next(
        (
            pair
            for pair in homepage_pairs
            if isinstance(pair, dict)
            and pair.get("english") == "README.md"
            and pair.get("chinese") == "README_ZH.md"
        ),
        None,
    )
    if homepage is None:
        return ["Markdown audit policy: missing README.md/README_ZH.md single-product homepage pair"]

    errors: list[str] = []
    if homepage.get("profile") != "single-product":
        errors.append("Markdown audit policy: README homepage profile must be single-product")
    components = homepage.get("required_components")
    if not isinstance(components, list) or not REQUIRED_HOMEPAGE_COMPONENTS.issubset(components):
        errors.append("Markdown audit policy: README homepage must require its complete visual header and h2 components")
    quick_links = homepage.get("required_quick_links")
    if not isinstance(quick_links, list) or not REQUIRED_HOMEPAGE_QUICK_LINKS.issubset(quick_links):
        errors.append("Markdown audit policy: README homepage must require product, documentation, firmware, and esp_idf quick links")
    badges = homepage.get("required_badges")
    if not isinstance(badges, list) or not REQUIRED_HOMEPAGE_BADGES.issubset(badges):
        errors.append("Markdown audit policy: README homepage must require build and license badges")
    if homepage.get("required_h2_icons") != REQUIRED_HOMEPAGE_H2_ICONS:
        errors.append("Markdown audit policy: README homepage h2 icon order must match the bilingual homepage")
    if homepage.get("h3_emoji_allow_patterns") != []:
        errors.append("Markdown audit policy: README homepage h3 headings must use the default no-emoji policy")
    return errors


def required_ci_helper_errors(repo_root: Path) -> list[str]:
    return [
        f"missing CI helper: {relative}"
        for relative in REQUIRED_CI_HELPERS
        if not (repo_root / relative).is_file()
    ]


def bundled_arduino_library_errors(repo_root: Path) -> list[str]:
    """Keep the reviewed Arduino libraries complete and version-identifiable."""
    errors: list[str] = []
    root = repo_root / "examples" / "arduino" / "libraries"
    for name, requirement in ARDUINO_LIBRARY_REQUIREMENTS.items():
        library = root / name
        properties = library / "library.properties"
        try:
            metadata = properties.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{properties.relative_to(repo_root)}: cannot read bundled library metadata: {error}")
            continue
        version = re.search(r"(?m)^version=([^\r\n]+)$", metadata)
        expected_version = requirement["version"]
        if version is None or version.group(1) != expected_version:
            found = version.group(1) if version else "missing"
            errors.append(
                f"{properties.relative_to(repo_root)}: expected bundled version "
                f"{expected_version}, found {found}"
            )
        for relative in requirement["files"]:
            required = library / relative
            if not required.is_file():
                errors.append(f"{required.relative_to(repo_root)}: missing bundled library file")

    lv_conf = root / "lv_conf.h"
    try:
        configuration = lv_conf.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"{lv_conf.relative_to(repo_root)}: cannot read bundled LVGL configuration: {error}")
    else:
        for marker in ARDUINO_LV_CONF_MARKERS:
            if marker not in configuration:
                errors.append(f"{lv_conf.relative_to(repo_root)}: missing full configuration marker {marker}")
    return errors


def repository_errors(repo_root: Path = REPO_ROOT) -> list[str]:
    errors: list[str] = []
    examples = direct_examples(repo_root)
    if len(examples) != 12:
        errors.append(f"expected 12 direct ESP-IDF product examples, found {len(examples)}")

    for english, chinese in REQUIRED_PAIRS:
        if not (repo_root / english).is_file():
            errors.append(f"missing required document: {english}")
        if not (repo_root / chinese).is_file():
            errors.append(f"missing required document: {chinese}")

    for relative in REQUIRED_CI_DEFAULTS:
        if not (repo_root / relative).is_file():
            errors.append(f"missing CI sdkconfig overlay: {relative}")

    errors.extend(required_ci_helper_errors(repo_root))

    errors.extend(bundled_arduino_library_errors(repo_root))

    errors.extend(bsp_pin_policy_errors(repo_root))

    audit_policy = repo_root / ".github" / "markdown-audit.json"
    try:
        policy = json.loads(audit_policy.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        errors.append(f"invalid Markdown audit policy: {error}")
    else:
        errors.extend(homepage_policy_errors(policy))

    for example in examples:
        defaults = example / "sdkconfig.defaults"
        relative = defaults.relative_to(repo_root).as_posix()
        if not defaults.is_file():
            errors.append(f"{relative}: missing product sdkconfig.defaults")
            continue
        text = defaults.read_text(encoding="utf-8")
        errors.extend(flash_size_selection_errors(text, relative))
        errors.extend(duplicate_sdkconfig_assignment_errors(text, relative))
        for stale_key in ("CONFIG_BSP_LCD_TYPE_HDMI", "CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A"):
            if stale_key in text:
                errors.append(f"{relative}: stale non-product setting {stale_key}")

    errors.extend(revision_profile_errors(repo_root, examples))

    wifi_source = (
        repo_root / "examples" / "esp-idf" / "04_wifistation" / "main" / "station_example_main.c"
    ).read_text(encoding="utf-8")
    errors.extend(wifi_public_log_errors(wifi_source))

    audio_source = (
        repo_root / "examples" / "esp-idf" / "06_I2SCodec" / "main" / "i2s_es8311_example.c"
    ).read_text(encoding="utf-8")
    audio_config = (
        repo_root / "examples" / "esp-idf" / "06_I2SCodec" / "main" / "example_config.h"
    ).read_text(encoding="utf-8")
    audio_overlay = (repo_root / "config" / "ci" / "i2s_echo.defaults").read_text(encoding="utf-8")
    errors.extend(audio_bsp_contract_errors(audio_source, audio_overlay, audio_config))

    player_source = (
        repo_root / "examples" / "esp-idf" / "10_mp4_player" / "main" / "main.c"
    ).read_text(encoding="utf-8")
    for required in (
        "#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888",
        "APP_STREAM_JPEG_CONFIG_DEFAULT_RGB565()",
        "APP_STREAM_JPEG_CONFIG_DEFAULT_RGB888()",
        "DISPLAY_BYTES_PER_PIXEL",
    ):
        if required not in player_source:
            errors.append(f"10_mp4_player: missing conditional display path {required}")

    player_manifest = (
        repo_root / "examples" / "esp-idf" / "10_mp4_player" / "main" / "idf_component.yml"
    ).read_text(encoding="utf-8")
    if 'version: ">=2.3.0,<2.6.0"' not in player_manifest:
        errors.append(
            "10_mp4_player: esp_audio_codec must stay below v2.6 for pre-revision-3 P4 hardware"
        )

    usb_main_cmake = (
        repo_root / "examples" / "esp-idf" / "12_usb_extend_screen" / "main" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    if usb_main_cmake.count('"app_uac.c"') != 1:
        errors.append("12_usb_extend_screen: app_uac.c must appear once in its conditional source list")
    usb_project_cmake = (
        repo_root / "examples" / "esp-idf" / "12_usb_extend_screen" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    usb_manifest = (
        repo_root
        / "examples"
        / "esp-idf"
        / "12_usb_extend_screen"
        / "main"
        / "idf_component.yml"
    ).read_text(encoding="utf-8")
    usb_tinyusb_config = (
        repo_root
        / "examples"
        / "esp-idf"
        / "12_usb_extend_screen"
        / "main"
        / "tusb"
        / "tusb_config_uac.h"
    ).read_text(encoding="utf-8")
    errors.extend(
        usb_audio_dependency_errors(
            usb_manifest, usb_main_cmake, usb_project_cmake, usb_tinyusb_config
        )
    )
    usb_main = (
        repo_root
        / "examples"
        / "esp-idf"
        / "12_usb_extend_screen"
        / "main"
        / "usb_extend_screen.c"
    ).read_text(encoding="utf-8")
    if "app_uac_init" in usb_main:
        errors.append("12_usb_extend_screen: UAC initialization must be owned by app_usb only")

    brookesia_cmake = (
        repo_root
        / "examples"
        / "esp-idf"
        / "11_esp_brookesia_phone"
        / "components"
        / "brookesia_core"
        / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    coze_source = (
        repo_root
        / "examples"
        / "esp-idf"
        / "11_esp_brookesia_phone"
        / "components"
        / "brookesia_core"
        / "ai_framework"
        / "agent"
        / "coze_chat_app.cpp"
    ).read_text(encoding="utf-8")
    errors.extend(brookesia_ai_compatibility_errors(brookesia_cmake, coze_source))

    errors.extend(factory_firmware_integrity_errors(repo_root))

    errors.extend(local_link_errors(repo_root))
    return errors


def main() -> int:
    errors = repository_errors()
    if errors:
        print("Repository audit failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(
        f"Repository audit passed: {len(direct_examples())} product examples, "
        f"{len(markdown_files())} Markdown files, local links valid."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
