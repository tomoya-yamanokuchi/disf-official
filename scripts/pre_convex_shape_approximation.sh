#!/bin/bash

# 処理するオブジェクトのリスト
objects=(
    "006_mustard_bottle"
    "011_banana"
    "029_plate"
    "033_spatula"
    "035_power_drill"
    "037_scissors"
    "042_adjustable_wrench"
    "048_hammer"
    "058_golf_ball"
    "065-j_cups"
)

# 各オブジェクトに対して `obj2mjcf` を実行
for obj in "${objects[@]}"; do
    echo "Processing $obj..."
    obj2mjcf --obj-dir ../models/ycb/"$obj"/tsdf --save-mjcf --decompose --add-free-joint --coacd-args.preprocess-resolution 50
done

echo "All objects processed."
