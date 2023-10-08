# !/bin/bash

./cleanup.sh

python ./generate_png_from_csv.py adaptive_tap_frames.csv

cd ./oi4v
ffmpeg -hide_banner -f image2 -framerate 24 -i iter_frame_%d.png -loop 0 anim_output.gif
cd ..