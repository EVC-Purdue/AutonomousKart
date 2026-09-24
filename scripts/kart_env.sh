# Source for a kart shell: ROS, venv, workspace, DDS and the kart command.

KART_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export KART_WS

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# Native venv lives in the workspace, the container's outside the bind mount.
for _v in "${KART_VENV:-}" "$KART_WS/.venv" "$HOME/.kart-venv"; do
  if [ -n "$_v" ] && [ -f "$_v/bin/activate" ]; then
    source "$_v/bin/activate"
    break
  fi
done
unset _v

[ -f "$KART_WS/install/setup.bash" ] && source "$KART_WS/install/setup.bash"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$KART_WS/docker/cyclonedds.xml"
[ -d /usr/local/cuda/bin ] && export PATH="/usr/local/cuda/bin:$PATH"

kart() { "$KART_WS/scripts/kart" "$@"; }
