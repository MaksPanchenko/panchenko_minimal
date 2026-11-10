#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
STATE="$ROOT/.runtime"
mkdir -p "$STATE"

fail() { printf '%s\n' "$*" >&2; exit 1; }
as_root() {
    if (( EUID == 0 )); then "$@"; else sudo -- "$@"; fi
}

prepare_only=false
rebuild=false
arguments=()
for argument in "$@"; do
    case "$argument" in
        --prepare-only) prepare_only=true ;;
        --rebuild) rebuild=true ;;
        --help|-h)
            printf '%s\n' 'Запуск: ./run.sh [device:=/dev/ttyACM0] [baudrate:=115200]' \
                '  --prepare-only  Установить зависимости и собрать без подключения контроллера.' \
                '  --rebuild       Повторно выполнить сборку.'
            exit 0 ;;
        *) arguments+=("$argument") ;;
    esac
done

source /etc/os-release
[[ "$ID" == ubuntu ]] || fail 'Поддерживаются Ubuntu 22.04 и 24.04.'
case "$VERSION_ID" in
    22.04) distro=humble ;;
    24.04) distro=jazzy ;;
    *) fail 'Поддерживаются Ubuntu 22.04 и 24.04.' ;;
esac
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == "$distro" ]] || \
    fail "Для Ubuntu $VERSION_ID используется $distro. Запустите скрипт в терминале без окружения $ROS_DISTRO."
architecture="$(dpkg --print-architecture)"
[[ "$architecture" == amd64 || "$architecture" == arm64 ]] || fail 'Нужна архитектура amd64 или arm64.'

packages=(build-essential cmake libasio-dev libeigen3-dev python3-colcon-common-extensions
    "ros-$distro-ros-base" "ros-$distro-mavros-msgs" "ros-$distro-tf2-ros")
signature="$(printf '%s\n' "$ROOT" "$VERSION_ID" "$architecture" "$distro" "${packages[@]}" \
    "$(sha256sum run.sh package.xml)" | sha256sum | cut -d' ' -f1)"
dependency_state="$STATE/dependencies.state"

exec 9>"$STATE/setup.lock"
flock 9
if [[ ! -f "$dependency_state" || "$(<"$dependency_state")" != "$signature" ||
      ! -r "/opt/ros/$distro/setup.bash" || ! -f "/opt/ros/$distro/share/mavros_msgs/package.xml" ||
      ! -f /usr/include/asio.hpp || ! -x /usr/bin/colcon || ! -x /usr/bin/g++ ]]; then
    missing=()
    for package in "${packages[@]}"; do
        if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != 'install ok installed' ]]; then
            missing+=("$package")
        fi
    done
    if (( ${#missing[@]} )); then
        printf 'Устанавливаю зависимости для ROS 2 %s...\n' "$distro"
        (( EUID == 0 )) || command -v sudo >/dev/null || fail 'Для установки зависимостей нужен sudo.'
        as_root apt-get update
        if ! apt-cache show "ros-$distro-ros-base" >/dev/null 2>&1; then
            as_root apt-get install -y --no-install-recommends ca-certificates curl software-properties-common
            as_root add-apt-repository -y -n universe
            if [[ "$VERSION_ID" == 24.04 && -f /etc/apt/sources.list.d/ubuntu.sources ]]; then
                as_root python3 - /etc/apt/sources.list.d/ubuntu.sources <<'PY'
from pathlib import Path
import re
import shutil
import sys

path = Path(sys.argv[1])
original = path.read_text()
def enable_pockets(match):
    fields = match.group().split()
    if 'noble' not in fields:
        return match.group()
    missing = [p for p in ('noble-updates', 'noble-backports') if p not in fields]
    return match.group() + ''.join(' ' + p for p in missing)
updated = re.sub(r'^Suites:[^\n]*', enable_pockets, original, flags=re.MULTILINE | re.IGNORECASE)
if updated != original:
    backup = path.with_suffix('.sources.bak')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(updated)
PY
            fi
            repository_package="$STATE/ros2-apt-source.deb"
            curl -fL --retry 3 -o "$repository_package" \
                "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.${VERSION_CODENAME}_all.deb"
            as_root apt-get install -y "$repository_package"
            as_root apt-get update
        fi
        as_root apt-get install -y --no-install-recommends "${missing[@]}"
    fi
    printf '%s\n' "$signature" > "$dependency_state.tmp"
    mv "$dependency_state.tmp" "$dependency_state"
fi

source "/opt/ros/$distro/setup.bash"
build_base="$ROOT/build/$distro-$architecture"
install_base="$ROOT/install/$distro-$architecture"
build_state="$STATE/build.state"
build_signature="$(
    {
        printf '%s\n' "$signature"
        sha256sum CMakeLists.txt package.xml
        find src include third_party/msp/src third_party/msp/inc launch cfg -type f -print0 | \
            sort -z | xargs -0 sha256sum
    } | sha256sum | cut -d' ' -f1
)"
if $rebuild || [[ ! -f "$build_state" || "$(<"$build_state")" != "$build_signature" ||
                  ! -x "$install_base/multiwii/lib/multiwii/multiwii_node" ]]; then
    cache="$build_base/multiwii/CMakeCache.txt"
    if [[ -f "$cache" ]] && ! grep -Fqx "CMAKE_HOME_DIRECTORY:INTERNAL=$ROOT" "$cache"; then
        rm -rf -- "$build_base" "$install_base"
    fi
    printf 'Собираю узел: ROS 2 %s, %s...\n' "$distro" "$architecture"
    export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
    export CC=/usr/bin/gcc
    export CXX=/usr/bin/g++
    colcon --log-base "$ROOT/log" build --base-paths "$ROOT" \
        --build-base "$build_base" --install-base "$install_base" --executor sequential \
        --event-handlers console_direct+ desktop_notification- \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    printf '%s\n' "$build_signature" > "$build_state.tmp"
    mv "$build_state.tmp" "$build_state"
else
    printf 'Использую готовую сборку: ROS 2 %s, %s.\n' "$distro" "$architecture"
fi
flock -u 9
exec 9>&-
source "$install_base/local_setup.bash"
$prepare_only && exit 0

device=''
for argument in "${arguments[@]}"; do
    case "$argument" in device:=*) device="${argument#device:=}" ;; esac
done
if [[ -z "$device" ]]; then
    shopt -s nullglob
    candidates=(/dev/serial/by-id/*)
    (( ${#candidates[@]} )) || candidates=(/dev/ttyACM* /dev/ttyUSB*)
    (( ${#candidates[@]} == 1 )) || fail 'Укажите порт контроллера: ./run.sh device:=/dev/ttyACM0'
    device="${candidates[0]}"
    arguments+=("device:=$device")
fi
[[ -c "$device" ]] || fail "Не найден последовательный порт: $device"
if [[ ! -r "$device" || ! -w "$device" ]]; then
    device_group="$(stat -Lc '%G' "$device")"
    if (( EUID != 0 )) && [[ "$device_group" == dialout || "$device_group" == uucp ]] &&
       [[ "$(id -g)" != "$(stat -Lc '%g' "$device")" ]]; then
        printf 'Для доступа к %s нужен запуск с группой %s.\n' "$device" "$device_group"
        exec sudo -u "$(id -un)" -g "$device_group" -- "$ROOT/run.sh" "${arguments[@]}"
    fi
    fail "Нет доступа на чтение и запись: $device"
fi

export ROS_LOG_DIR="$STATE/ros-log"
mkdir -p "$ROS_LOG_DIR"
printf 'Запускаю узел: %s. Остановка: Ctrl+C.\n' "$device"
exec ros2 launch multiwii multiwii.launch.py "${arguments[@]}"
