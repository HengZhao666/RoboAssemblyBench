# RoboAssemblyBench 本机部署进度

记录时间：2026-09-11 23:20（UTC+8）  
机器：`user-Standard-PC-i440FX-PIIX-1996`（Ubuntu 22.04，NVIDIA L20）

下次 SSH / Cursor 登录后，先打开本文件，再按下面路径核对。

---

## 当前状态

**第一条无头 demo 已成功结束。** 残留的 `conda run` 包装进程（PID 50672 / 50701 / 50706）已于 23:19 关掉。当前没有 `generate_demos.py` 在跑，GPU 显存约 325 MiB。

| 项 | 值 |
|---|---|
| 配方 | `fabrica_plumbers_block_ur5e_right_base_prepare` |
| 结果 | `success: true`，`failed: false`，`phase_status: success` |
| 步数 | 16676（seed 0） |
| 视频时间 | 2026-09-11 23:01–23:03 |
| 输出目录 | `/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/` |
| Kit 日志 | `/home/user/isaacsim-5.1.0/kit/logs/Kit/Isaac-Sim Python/5.1/kit_20260911_222014.log` |

本机生成的视频（约 47M）：

```text
/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/episode_0000_live_videos/
  observation_images_front.mp4          15M
  observation_images_left_wrist.mp4     17M
  observation_images_right_wrist.mp4    16M
```

核对命令：

```bash
pgrep -af generate_demos.py   # 应无输出
python3 -m json.tool \
  /home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/collect_results.json | head -20
ls -lh /home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/episode_0000_live_videos/
```

官方下载的对照视频在：

```text
/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_ur5e_right_base_prepare_demo/episode_0000_live_videos/
```

---

## 已经完成的工作

1. **GPU 驱动**  
   重启后 `nvidia-smi` 正常：L20，驱动 `580.178.04`。

2. **Isaac Sim 5.1.0**（不要和 6.0.1 搞混）  
   - 5.1：`/home/user/isaacsim-5.1.0`（约 17G，`isaac-sim.sh` 在此）  
   - 6.0.1 仍在：`/home/user/isaacsim`（未覆盖）

3. **代码仓库**  
   `/home/user/RoboAssemblyBench`

4. **Conda 环境**  
   名称：`internutopia311`  
   路径：`/Embodied_AI/miniconda3/envs/internutopia311`  
   激活时会自动设置 `ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0`  
   已能导入：`internutopia`、`torch 2.14.0+cu130`（CUDA 可用）、`pyarrow`

5. **Hugging Face 资源**（官方站不通，用的镜像 `https://hf-mirror.com`）  
   - 官方预览视频：  
     `/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_ur5e_right_base_prepare_demo/`  
   - 七个 canonical 任务包：  
     `/home/user/RoboAssemblyBench/roboassemblybench/assets/Fabrica/canonical_7_bundles/task_bundles/`

6. **无头冒烟（已成功）**  
   输出目录（**不要和官方预览目录搞混**）：  
   `/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/`  
   第一次因缺 `pyarrow` 失败；安装后重跑成功。`conda run` 会缓冲日志并可能残留外壳进程，下次长任务请用 `python -u` + `tmux`/`screen`/`nohup`。

---

## 下次登录：环境怎么激活

```bash
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
echo "$ISAAC_SIM_ROOT"    # 应为 /home/user/isaacsim-5.1.0
cd /home/user/RoboAssemblyBench
```

Hugging Face 镜像（直连 huggingface.co 不通）：

```bash
export HF_ENDPOINT=https://hf-mirror.com
```

系统没装 apt 的 `git`，用的是 conda 里的 git：

```text
/Embodied_AI/miniconda3/bin/git
```

---

## 若要重跑同一条无头 demo

不会覆盖官方预览目录。建议先改 `OUTPUT_DIR`，避免覆盖这次成功结果。

```bash
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
cd /home/user/RoboAssemblyBench

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
export PYTHONNOUSERSITE=1
export HEADLESS=1 NUM_DEMOS=1 MAX_TRIALS=1 LIVE_VIDEO_FRAME_STRIDE=8
export OUTPUT_DIR=outputs/fabrica_plumbers_block_smoke

# 建议直接 python，避免 conda run 把日志缓冲住
python -u roboassemblybench/scripts/generate_demos.py \
  --worker-mode collect \
  --worker-recipe fabrica_plumbers_block_ur5e_right_base_prepare \
  --worker-scene-profile taoyuan_grscenes_tabletop \
  --worker-results-path "${OUTPUT_DIR}/collect_results.json" \
  --worker-seeds 0 \
  --max-trials 1 \
  --start-seed 0 \
  --record-live-video \
  --live-video-fps 30 \
  --live-video-frame-stride 8 \
  --output-dir "${OUTPUT_DIR}" \
  --headless \
  --skip-episode-steps
```

建议用 `tmux` / `screen` 或 `nohup ... &`，避免再断开会话时进程被杀。

Isaac 进度在 Kit 日志：

```text
/home/user/isaacsim-5.1.0/kit/logs/Kit/Isaac-Sim Python/5.1/
```

搜 `[rollout-progress]`、`Traceback`。

---

## 重要路径速查

| 用途 | 路径 |
|---|---|
| 仓库 | `/home/user/RoboAssemblyBench` |
| Isaac 5.1 | `/home/user/isaacsim-5.1.0` |
| Isaac 6.0（旧，勿用于本仓库） | `/home/user/isaacsim` |
| Conda 环境 | `/Embodied_AI/miniconda3/envs/internutopia311` |
| 本次冒烟输出 | `/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_smoke/` |
| 官方 HF 预览 | `/home/user/RoboAssemblyBench/outputs/fabrica_plumbers_block_ur5e_right_base_prepare_demo/` |
| 7 任务资源 | `/home/user/RoboAssemblyBench/roboassemblybench/assets/Fabrica/canonical_7_bundles/task_bundles/` |
| 本说明 | `/home/user/RoboAssemblyBench/RoboAssemblyBench部署进度.md` |

尚未做：六任务 canonical 复现、2000 条采集、ACT 训练与评测。
