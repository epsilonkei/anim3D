#!/usr/bin/env bash
# $1 is start_number
# $2 is video title

# ffmpeg -framerate 30 -start_number 0 -i /tmp/water/%3d.ppm -vcodec mpeg4 $1
ffmpeg -framerate 30 -start_number 0 -i /tmp/water/%3d.ppm -vcodec mpeg4 -b:v 20M $1
