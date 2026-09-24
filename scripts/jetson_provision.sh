#!/usr/bin/env bash
# Provision a Jetson to run the kart without the devcontainer.
#
# Mirrors docker/Dockerfile so the host ends up with the same ROS packages and
# the same pinned Python environment. Run it as the normal user
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${WS:-$HOME/AutonomousKart}"
VENV="$WS/.venv"

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------- preflight
say "Preflight"
. /etc/os-release
echo "    os        $PRETTY_NAME ($VERSION_CODENAME, $(dpkg --print-architecture))"
if [ "$VERSION_CODENAME" != "jammy" ]; then
  echo "    ROS 2 $ROS_DISTRO targets Ubuntu 22.04 (jammy); this is $VERSION_CODENAME." >&2
  echo "    Refusing to guess. Stop here." >&2
  exit 1
fi
[ -d "$WS" ] || { echo "    no workspace at $WS" >&2; exit 1; }
echo "    workspace $WS"
if have nvcc; then
  echo "    nvcc      $(nvcc --version | sed -n 's/.*release \([0-9.]*\).*/\1/p')"
elif [ -x /usr/local/cuda/bin/nvcc ]; then
  echo "    nvcc      /usr/local/cuda/bin/nvcc (not on PATH, will be added)"
else
  echo "    nvcc      NOT FOUND - the CUDA MPC solver will not build" >&2
fi
echo "    free disk $(df -h "$WS" | awk 'NR==2{print $4}')"

# Validate sudo
if [ -t 0 ]; then
  sudo -v
else
  sudo -S -v
fi

say "ROS 2 apt repository"
if [ -f /etc/apt/sources.list.d/ros2.list ] && [ -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  echo "    already configured"
else
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends software-properties-common curl gnupg ca-certificates
  sudo add-apt-repository -y universe
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  echo "    added"
fi

# apt packages
# Same set as docker/Dockerfile
say "Packages"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${ROS_DISTRO}-ackermann-msgs" \
  "ros-${ROS_DISTRO}-rclpy" \
  "ros-${ROS_DISTRO}-ros2bag" \
  "ros-${ROS_DISTRO}-tf2-tools" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  python3-pip python3-colcon-common-extensions python3.10-venv \
  python3-rosdep \
  git curl nano socat rtklib i2c-tools build-essential

# rosdep
say "rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update || echo "    rosdep update failed (non-fatal)"

# workspace path
say "Workspace path"
echo "    $WS"
mkdir -p "$WS/logs"

# Anything the container wrote is owned by root, because it ran as root over the
# bind mount. colcon cannot write its log directory until that is undone.
if find "$WS" -maxdepth 1 \( -name build -o -name install -o -name log -o -name logs \) -user root 2>/dev/null | grep -q .; then
  say "Reclaiming container-owned build directories"
  sudo chown -R "$(id -u):$(id -g)" "$WS"/build "$WS"/install "$WS"/log "$WS"/logs 2>/dev/null || true
  echo "    done"
fi

# venv
say "Python environment"
if [ -d "$VENV" ] && [ ! -f "$VENV/.hermetic" ]; then
  echo "    replacing $VENV (was built with --system-site-packages)"
  rm -rf "$VENV"
fi
if [ ! -d "$VENV" ]; then
  python3 -m venv "$VENV"
  touch "$VENV/.hermetic"
  echo "    created $VENV"
else
  echo "    reusing $VENV"
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --upgrade pip wheel
# Both files in ONE resolver pass, so a conflict between a runtime pin and a
# tooling pin fails loudly here instead of resolving differently per install.
PYTHONNOUSERSITE=1 python -m pip install \
  -r "$WS/requirements.txt" -r "$WS/requirements-dev.txt"
if [ -f "$WS/requirements.local.txt" ]; then
  PYTHONNOUSERSITE=1 python -m pip install -r "$WS/requirements.local.txt" || true
fi

# Let the venv see ROS's own python packages, same trick as the Dockerfile.
cat > "$VENV/lib/python3.10/site-packages/ros2.pth" <<EOF
/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages
EOF

# For cython
touch "$VENV/COLCON_IGNORE"

# cuda solver
say "CUDA MPC solver"
bash "$WS/scripts/build_mpc_cuda.sh" || echo "    build failed (non-fatal, numpy fallback)"

# env helper
say "Environment helper"
if ! grep -q "kart_env.sh" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<EOF

# Kart: ROS 2, the venv, the workspace and the \`kart\` command.
[ -f "$WS/scripts/kart_env.sh" ] && . "$WS/scripts/kart_env.sh"
EOF
  echo "    added kart_env.sh to ~/.bashrc"
else
  echo "    ~/.bashrc already sources kart_env.sh"
fi

# verify
say "Verify"
set +u
# shellcheck disable=SC1091
source "$WS/scripts/kart_env.sh"
set -u
echo "    ros2        ${ROS_DISTRO}, $(ros2 pkg list 2>/dev/null | wc -l) packages"
echo "    colcon      $(colcon version-check 2>/dev/null | head -1 || command -v colcon || echo FAILED)"
echo "    python      $(python -V 2>&1)"
python - <<'PY'
import importlib, sys
for mod in ("numpy", "scipy", "cv2", "rclpy", "serial", "can", "flask", "sklearn", "joblib"):
    try:
        m = importlib.import_module(mod)
        print(f"    {mod:<11} {getattr(m, '__version__', 'ok')}")
    except Exception as exc:
        print(f"    {mod:<11} MISSING ({exc.__class__.__name__})", file=sys.stderr)
PY
if command -v nvcc >/dev/null 2>&1; then
  echo "    nvcc        $(nvcc --version | sed -n 's/.*release \([0-9.]*\).*/\1/p')"
else
  echo "    nvcc        NOT ON PATH" >&2
fi
# colcon has to be the venv's, or every generated entry point gets a
# system-python shebang and the nodes cannot import their dependencies.
case "$(command -v colcon)" in
  "$VENV"/*) echo "    colcon      $(command -v colcon)" ;;
  *)         echo "    colcon      $(command -v colcon) NOT THE VENV'S - nodes will fail to import" >&2 ;;
esac

say "Build and test"
( cd "$WS" && colcon build --packages-select autonomous_kart 2>&1 | tail -2 )
echo "    entry point shebang: $(head -1 "$WS/install/autonomous_kart/lib/autonomous_kart/master_api" 2>/dev/null || echo MISSING)"
( cd "$WS" && python -m pytest src/autonomous_kart/test/ -q 2>&1 | tail -2 )

say "Done"
cat <<EOF
    Next:
      source $WS/scripts/kart_env.sh
      cd $WS && colcon build --packages-select autonomous_kart
      source $WS/install/setup.bash
EOF
