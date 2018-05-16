#!/usr/bin/env bash

if [ ! "$XRANGE" ]; then XRANGE="[*:*]"; fi
if [ ! "$YRANGE" ]; then YRANGE="[*:*]"; fi

echo "XRANGE=$XRANGE";
echo "YRANGE=$YRANGE";

# gnuplot <<EOF
# set terminal postscript eps color enhanced
# set tics font "Times New Roman,25"
# set xlabel font "Times New Roman,25"
# set ylabel font "Times New Roman,25"
# set zlabel font "Times New Roman,25"
# set key font "Times New Roman,20"
# set key right top
# set key width 8
# set output "BH.eps"
# set grid
# set size ratio 0.5
# set xlabel "Number of particles"
# set ylabel "Average elapsed time[s]"
# set xrange ${XRANGE}
# set yrange ${YRANGE}
# set title "_"
# plot "~/tmp/BH_0.5.dat" using 1:2 with line linewidth 3 title "Basic Implementation",
#      "~/tmp/BH_0.5.dat" using 1:3 with line linewidth 3 title "Barnes-Hut algorithm",
# EOF

gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"
set xlabel font "Times New Roman,25"
set ylabel font "Times New Roman,25"
set zlabel font "Times New Roman,25"
set key font "Times New Roman,20"
set key right top
set key width 8
set output "newton_cradle_energy.eps"
set grid
set size ratio 0.5
set xlabel "Time[s]"
set ylabel "Total particles' energy"
set xrange ${XRANGE}
set yrange ${YRANGE}
set title "_"
plot "/tmp/newton_cradle_energy.dat" using 1:2 with line linewidth 3 title "Total energy"
EOF

sed -i "s/^\%\%BoundingBox\: .\+$/%%BoundingBox: 50 80 410 272/g" *.eps;
