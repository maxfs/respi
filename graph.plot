set datafile separator “,”
set autoscale fix
set key outside right center

set title “Title”
plot ‘filename.csv’ using 1 title “LineTitle” with lines

pause -1