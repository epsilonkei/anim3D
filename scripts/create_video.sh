#!/usr/bin/env bash
# $1 is start_number

ffmpeg -framerate 30 -start_number $1 -i /tmp/water/%d.ppm -vcodec mpeg4 $2
