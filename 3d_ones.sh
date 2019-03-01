#! /bin/bash

cd ./build

# Reconstruct every record separately
for i in 1 2 3 4 6 7 8 9 10 11 12 13 14; do
echo "Processing record: $i ..."
./bin/3d_recon --records="$i" --pairs_look_back=4 --matches_num_thresh=7 \
 --matches_line_dist_thresh=10.0 --sfm_repr_error_thresh=10.0            \
 --sfm_max_merge_dist=5.0 --noviz --output=sfm_out_m_one_$i.bin          \
 > log_output_$i.txt
done