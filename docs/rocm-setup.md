# ROCm 环境配置记录

## 硬件

- **GPU**: AMD Radeon 780M (gfx1103, RDNA3)
- **CPU**: AMD Ryzen 9 7940HS
- **系统**: Ubuntu 22.04, ROCm 6.4.2

## 安装流程

### 1. 安装 ROCm 运行时

```bash
sudo apt install -y rocm-hip-runtime hipblas hipfft rocm-smi-lib rocblas miopen-hip
```

库路径：`/opt/rocm-6.4.2/lib/`

### 2. 添加用户到 GPU 组

```bash
sudo usermod -aG render,video $USER
# 需要重新登录生效
```

### 3. 安装 ONNX Runtime ROCm

```bash
pip install onnxruntime-rocm
# 仅 1.22.2.post1 可用
```

## 环境变量

```bash
export LD_LIBRARY_PATH=/opt/rocm-6.4.2/lib:$LD_LIBRARY_PATH
export HSA_OVERRIDE_GFX_VERSION=11.0.2          # gfx1103 -> gfx1102 兼容
export ROCBLAS_TENSILE_LIBRARY_PATH=/opt/rocm-6.4.2/lib/rocblas/library/TensileLibrary_lazy_gfx1102.dat
```

## 当前状态

| 组件 | 状态 | 说明 |
|------|------|------|
| ROCm 运行时 | ✅ | HIP init/malloc/free 正常 |
| rocminfo | ✅ | 检测到 gfx1103 |
| rocBLAS | ✅ | 需手动指定 TensileLibrary |
| ONNX Runtime ROCm | ❌ | Session 创建成功，推理时 crash |
| ONNX Runtime CPU | ✅ | 80 FPS，满足 20Hz 跟踪 |

## 已知问题

**onnxruntime-rocm 1.22.2 与 ROCm 6.4.2 ABI 不兼容**

- `hip_global.cpp:158 Module not initialized` — 推理时崩溃
- PyPI 仅提供 1.22.2.post1 一个版本
- 根因：onnxruntime-rocm 1.22 编译时链接 ROCm ≤6.2，与 6.4 的 HIP runtime 不兼容

**修复方案：**
1. 降级 ROCm → 6.2/6.3（可能影响其他包）
2. 从源码编译 onnxruntime（`./build.sh --use_rocm --rocm_home=/opt/rocm-6.4.2`）
3. 等待 onnxruntime-rocm 发布新版 wheel

## YOLOv12n 性能

| Provider | 延迟 | FPS |
|----------|------|-----|
| CPU (7940HS) | 12.4ms | 80 |
| ROCm (目标) | ~3-5ms | 200+ |

当前 CPU 80 FPS 已远超 `target_tracker_node` 的 20Hz 处理频率，GPU 加速非阻塞项。
