#!/usr/bin/env bash
# Coarse one-factor-at-a-time ORB-SLAM3 RGB-D sweep on a bag, tabulating the
# objective tuple (ATE-vs-O3D, coverage %, track-loss gaps). Median-of-REPS.
#
# Usage: sweep.sh BAG OUTDIR [DURATION_S] [REPS]
#   DURATION_S=0 -> full bag.  Edit GRID below to change the axes.
set -eo pipefail
BAG="$1"; OUTDIR="$2"; DUR="${3:-60}"; REPS="${4:-2}"
mkdir -p "$OUTDIR"

# label  thdepth  nfeatures   (axis 1: ThDepth from depth histogram; axis 2: nFeatures)
GRID=(
  "th40_f1250 40 1250"
  "th60_f1250 60 1250"
  "th80_f1250 80 1250"
  "th80_f2000 80 2000"
  "th100_f2000 100 2000"
)

for spec in "${GRID[@]}"; do
  set -- $spec; LBL=$1; TH=$2; NF=$3
  CFG="$OUTDIR/cfg_${LBL}.yaml"
  python3 /scripts/make_orbbec_config.py --bag "$BAG" --out "$CFG" --thdepth "$TH" --nfeatures "$NF" --rgb 0
  for r in $(seq 1 "$REPS"); do
    bash /scripts/run_orbslam_rgbd.sh "$BAG" "$CFG" "$OUTDIR" "${LBL}_r${r}" 0.5 "$DUR" || echo "[sweep] FAILED ${LBL}_r${r}"
  done
done

echo "=== SWEEP SUMMARY (${BAG##*/}) ==="
python3 - "$OUTDIR" <<'PY'
import json, glob, os, sys, statistics
outdir = sys.argv[1]
rows = {}
for f in sorted(glob.glob(os.path.join(outdir, "*.eval.json"))):
    d = json.load(open(f))
    lbl = d["label"].rsplit("_r", 1)[0]
    rows.setdefault(lbl, []).append(d)
def med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None
print(f"{'config':<14} {'ATE_med(mm)':>11} {'cov%_med':>9} {'losses':>7} {'reps':>5}")
summ = []
for lbl, ds in rows.items():
    ate = med([x["ate_rmse_vs_o3d"] for x in ds])
    cov = med([x["coverage"] for x in ds])
    los = med([x["track_loss_gaps"] for x in ds])
    summ.append((lbl, ate, cov, los, len(ds)))
# rank: coverage first (desc), then ATE (asc)
summ.sort(key=lambda r: (-(r[2] or 0), r[1] if r[1] is not None else 9e9))
for lbl, ate, cov, los, n in summ:
    am = f"{ate*1000:.1f}" if ate is not None else "n/a"
    print(f"{lbl:<14} {am:>11} {(cov or 0)*100:>8.1f} {los if los is not None else '-':>7} {n:>5}")
PY
