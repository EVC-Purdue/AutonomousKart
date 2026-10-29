#!/usr/bin/env bash
# The same ~/.bashrc line the native install uses.
set -euo pipefail

grep -q "kart_env.sh" ~/.bashrc 2>/dev/null || cat >> ~/.bashrc <<'RC'

# Kart: ROS 2, the venv, the workspace and the `kart` command.
[ -f /ws/scripts/kart_env.sh ] && . /ws/scripts/kart_env.sh
RC
