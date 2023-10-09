# !/bin/bash

./cleanup.sh

python ./generate_png_from_csv.py adaptive_tap_frames.csv

cd ./oi4v
ffmpeg -i iter_frame_%d.png -vf palettegen palette.png
ffmpeg -i image_%02d.png -i palette.png -lavfi paletteuse video.gif
ffmpeg -hide_banner -thread_queue_size 1024 -f image2 -framerate 24 -i iter_frame_%d.png -i palette.png -lavfi paletteuse -loop 0 anim_output.gif
cd ..