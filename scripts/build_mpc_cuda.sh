#!/usr/bin/env bash
# Build the CUDA MPC solver to build/mpc_cuda/; falls back to numpy on error.
set -euo pipefail

WS="${KART_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
OUT="$WS/build/mpc_cuda"
SRC="$WS/cuda/mpc_cuda.cu"

NVCC="$(command -v nvcc || echo /usr/local/cuda/bin/nvcc)"
if [ ! -x "$NVCC" ]; then
  echo "no nvcc: skipping the CUDA solver, MPC will run on numpy" >&2
  exit 0
fi

# sm_87 is Jetson Orin.
ARCH="${MPC_CUDA_ARCH:-}"
if [ -z "$ARCH" ]; then
  case "$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo)" in
    *Orin*) ARCH=sm_87 ;;
    *)      ARCH=sm_87 ;;
  esac
fi

mkdir -p "$OUT"
# -cudart static: the container runtime injects libcuda.so.1, not libcudart.
"$NVCC" -DLIBRARY -O3 -arch="$ARCH" --shared -Xcompiler -fPIC \
        -cudart static -o "$OUT/libmpc_cuda.so" "$SRC" -I "$WS/cuda"
echo "built $OUT/libmpc_cuda.so ($ARCH)"
