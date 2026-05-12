#!/bin/bash
# ROCm 6.4.2 environment setup for 780M (gfx1103 → gfx1102 override)
# Source this BEFORE launching any GPU-using ROS2 nodes

export LD_LIBRARY_PATH=/opt/rocm-6.4.2/lib:$LD_LIBRARY_PATH
export HSA_OVERRIDE_GFX_VERSION=11.0.2
export ROCBLAS_TENSILE_LIBRARY_PATH=/opt/rocm-6.4.2/lib/rocblas/library/TensileLibrary_lazy_gfx1102.dat
export HIP_VISIBLE_DEVICES=0

# ONNX Runtime
export ORT_ROCM_DISABLE_TUNING=1

echo "ROCm 6.4.2 environment set (gfx1103 → gfx1102)"
echo "LD_LIBRARY_PATH includes: /opt/rocm-6.4.2/lib"
