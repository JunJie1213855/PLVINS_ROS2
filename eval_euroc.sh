#!/bin/bash
# ============================================================
# EuRoC 轨迹评估脚本
#
# 用法:
#   ./scripts/eval_euroc.sh <轨迹文件> <mav0目录> [--plot]
#
# 示例:
#   ./scripts/eval_euroc.sh KeyFrameTrajectory.txt ~/dataset/Euroc/V1_01_easy/mav0
#   ./scripts/eval_euroc.sh KeyFrameTrajectory.txt ~/dataset/Euroc/V1_01_easy/mav0 --plot
# ============================================================

set -e

if [ $# -lt 2 ]; then
    echo "用法: $0 <轨迹文件> <mav0目录> [--plot]"
    echo ""
    echo "  轨迹文件  - ORB_SLAM3 输出的 KeyFrameTrajectory.txt"
    echo "  mav0目录  - EuRoC 数据集的 mav0 目录"
    echo "  --plot     - 生成误差图 (需要显示器)"
    echo ""
    echo "示例:"
    echo "  $0 KeyFrameTrajectory.txt ~/dataset/Euroc/V1_01_easy/mav0"
    echo "  $0 KeyFrameTrajectory.txt ~/dataset/Euroc/V1_01_easy/mav0 --plot"
    exit 1
fi

TRAJ="$1"
MAV0="$2"
DO_PLOT=false
[ "$3" = "--plot" ] && DO_PLOT=true

if [ ! -f "$TRAJ" ]; then
    echo "[错误] 轨迹文件不存在: $TRAJ"
    exit 1
fi
if [ ! -f "$MAV0/state_groundtruth_estimate0/data.csv" ]; then
    echo "[错误] 真值文件不存在: $MAV0/state_groundtruth_estimate0/data.csv"
    exit 1
fi

# 场景名
SCENE=$(basename "$(dirname "$(dirname "$MAV0")")")
SEQ=$(basename "$(dirname "$MAV0")")
NAME="${SCENE}_${SEQ}"

WORK_DIR=$(dirname "$TRAJ")
RESULTS_DIR="$WORK_DIR/../results"
mkdir -p "$RESULTS_DIR"

GT_TUM="$WORK_DIR/${NAME}_gt.tum"
EST_TUM="$WORK_DIR/${NAME}_est.tum"
RESULT_ZIP="$RESULTS_DIR/${NAME}_ape.zip"

echo "=========================================="
echo "  EuRoC 轨迹评估"
echo "  场景: $SCENE / $SEQ"
echo "=========================================="

# 步骤1: 转换真值 → TUM
echo ""
echo "[1/3] 转换真值..."
python3 -c "
gt_path = '$MAV0/state_groundtruth_estimate0/data.csv'
tum_path = '$GT_TUM'
with open(gt_path) as f_in, open(tum_path, 'w') as f_out:
    for line in f_in:
        if line.startswith('#'):
            continue
        cols = line.strip().split(',')
        ts_ns = int(cols[0])
        ts_sec = ts_ns / 1e9
        tx, ty, tz = float(cols[1]), float(cols[2]), float(cols[3])
        qw, qx, qy, qz = float(cols[4]), float(cols[5]), float(cols[6]), float(cols[7])
        f_out.write(f'{ts_sec:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
"
echo "  真值: $(wc -l < "$GT_TUM") 条 → $GT_TUM"

# 步骤2: 转换估计轨迹
echo ""
echo "[2/3] 转换估计轨迹..."
python3 -c "
est_path = '$TRAJ'
tum_path = '$EST_TUM'
with open(est_path) as f_in, open(tum_path, 'w') as f_out:
    for line in f_in:
        line = line.strip()
        if not line:
            continue
        cols = line.split()
        ts_raw = float(cols[0])
        # auto-detect timestamp format
        if ts_raw > 1e12:
            ts_sec = ts_raw / 1e9               # nanoseconds → seconds
            # TUM format: ts_ns x y z qw qx qy qz → remap to qx qy qz qw
            f_out.write(f'{ts_sec:.9f} {cols[1]} {cols[2]} {cols[3]} {cols[5]} {cols[6]} {cols[7]} {cols[4]}\n')
        else:
            ts_sec = ts_raw                      # already seconds
            # EVO format: ts x y z qx qy qz qw → already TUM order, pass through
            f_out.write(f'{ts_sec:.9f} {cols[1]} {cols[2]} {cols[3]} {cols[4]} {cols[5]} {cols[6]} {cols[7]}\n')
"
echo "  估计: $(wc -l < "$EST_TUM") 条 → $EST_TUM"

# 步骤3: evo 评估
echo ""
echo "[3/3] evo_ape 评估..."

PLOT_PNG="$RESULTS_DIR/${NAME}_ape.png"
rm -f "$RESULT_ZIP"  # 避免交互式确认

EVO_CMD="evo_ape tum \"$GT_TUM\" \"$EST_TUM\" -va --save_results \"$RESULT_ZIP\" --save_plot \"$PLOT_PNG\" --no_warnings"

if $DO_PLOT && [ -n "${DISPLAY:-}" ]; then
    EVO_CMD="$EVO_CMD --plot"
fi

export MPLBACKEND=Agg
eval "$EVO_CMD" 2>&1

echo ""
echo "=========================================="
echo "  评估完成"
echo "  结果:  $RESULT_ZIP"
echo "  图表:  $PLOT_PNG"
echo "=========================================="
