#!/bin/sh
set -eu

n=5
script_dir=$(dirname "$0")
find ~/Documents/Datasets/gt_meshes/ -type d -name 'matterport3d_*' |
	while IFS= read -r sequence
	do
		sequence=${sequence##*_}
		"$script_dir"/repeat-roslaunch "$n" habitat.launch \
			'config:=$(find supereight_ros)/config/matterport3d/'"${sequence}.yaml"
	done
