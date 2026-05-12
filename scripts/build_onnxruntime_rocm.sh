#!/bin/bash
# Build onnxruntime from source with ROCm 6.4.2 support
# Fixes ABI incompatibility between onnxruntime-rocm wheel (built for ROCm ≤6.2) and ROCm 6.4.2
set -e

WORKDIR=${1:-/tmp/ort_build}
INSTALL_PREFIX=${2:-$HOME/.local}
NPROC=$(nproc)

echo "=== Building onnxruntime for ROCm 6.4.2 ==="
echo "Source: v1.22.2 (compatible with existing code)"
echo "Workdir: $WORKDIR"
echo "Install: $INSTALL_PREFIX"
echo "Cores: $NPROC"

# 1. Clone
if [ ! -d "$WORKDIR/onnxruntime" ]; then
    git clone --depth 1 --branch v1.22.2 --recursive \
        https://github.com/microsoft/onnxruntime.git "$WORKDIR/onnxruntime"
fi

cd "$WORKDIR/onnxruntime"

# 2. Build
./build.sh \
    --config Release \
    --use_rocm \
    --rocm_home=/opt/rocm-6.4.2 \
    --build_wheel \
    --parallel "$NPROC" \
    --skip_tests \
    --cmake_extra_defines \
        CMAKE_HIP_FLAGS="-D__HIP_PLATFORM_AMD__" \
        ONNX_RUNTIME_VERSION="1.22.2" \
        CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"

# 3. Install wheel
WHEEL=$(find build/Linux/Release/dist -name "onnxruntime_rocm-*.whl" | head -1)
if [ -n "$WHEEL" ]; then
    pip install --force-reinstall "$WHEEL"
    echo "=== Build Complete ==="
    echo "Wheel installed: $WHEEL"
else
    echo "ERROR: Wheel not found"
    exit 1
fi
