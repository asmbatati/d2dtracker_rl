#!/bin/bash
set -e
D="${MONTAGE_DIR:-montage_out}"; F="$D/frames"; OUT="$D/out"; mkdir -p "$OUT"; FONT="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
declare -a CLIPS=(
"static__sac_intercept_static_20000_steps|20k steps   early, still inefficient"
"static__sac_intercept_static_100000_steps|100k steps   tracks, just misses"
"static__model_static_refine_40k|Refined   clean capture"
"constant_velocity__model_cv_40k|Moving target   caught"
"evasive__model_evasive_40k|Evasive target   caught"
)
i=0
> "$OUT/list.txt"
for entry in "${CLIPS[@]}"; do
  dir="${entry%%|*}"; cap="${entry##*|}"
  n=$(ls "$F/$dir"/*.png 2>/dev/null | wc -l)
  [ "$n" -eq 0 ] && { echo "skip $dir (no frames)"; continue; }
  echo "clip $i: $dir ($n frames) -> \"$cap\""
  ffmpeg -y -loglevel error -framerate 30 -start_number 0 -i "$F/$dir/f_%05d.png" -frames:v 160     -vf "scale=1280:720,drawtext=fontfile=$FONT:text='$cap':fontcolor=white:fontsize=38:box=1:boxcolor=black@0.55:boxborderw=14:x=(w-text_w)/2:y=h-80,format=yuv420p"     -c:v libx264 -pix_fmt yuv420p -r 30 "$OUT/clip_$i.mp4"
  echo "file 'clip_$i.mp4'" >> "$OUT/list.txt"
  i=$((i+1))
done
ffmpeg -y -loglevel error -f concat -safe 0 -i "$OUT/list.txt" -c copy "$OUT/interception_training_montage.mp4"
echo "DONE"
