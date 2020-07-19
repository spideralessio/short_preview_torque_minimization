LONG=1
DT=0.001
FPS=60
USE_CACHE=1
P=100

if [ $USE_CACHE -eq 1 ]; then
	rm -rf imgs/*
	mv mats/mats_long mats_long
	mv mats/mats_short mats_short
	rm -rf mats
fi

METHODS=('MTN' 'MTND' 'MBP' 'MBPD' 'MTIWN' 'MTIWND' 'MBIWP' 'MBIWPD' 'MTSIWN' 'MTSIWND' 'MBSIWP' 'MBSIWPD')
LENGTH=${#METHODS[*]}
START=0
unset pids

if [ $USE_CACHE -eq 0 ]; then
    for (( i=$START; i<$LENGTH; i++ )); do
        METHOD=${METHODS[$i]}
        echo $METHOD
        matlab -nodisplay -nosplash -r "$METHOD($LONG, $DT, $P); exit(0)" &
        pids[${i}]=$!
    done

    # wait for all pids
    for pid in ${pids[*]}; do
        wait $pid
    done
else
	mv mats_long mats
fi

matlab -nodisplay -nosplash -r "gen_plot($LONG, $DT); exit(0)"
matlab -nodesktop -nosplash -r "gen_video($LONG, $DT, $FPS); exit(0)"

mv mats mats_long
mv imgs imgs_long
mkdir imgs
mkdir mats

LONG=0
unset pids
if [ $USE_CACHE -eq 0 ]; then
    for (( i=$START; i<$LENGTH; i++ )); do
        METHOD=${METHODS[$i]}
        echo $METHOD
        matlab -nodisplay -nosplash -r "$METHOD($LONG, $DT, $P); exit(0)" &
        pids[${i}]=$!
    done

    # wait for all pids
    for pid in ${pids[*]}; do
        wait $pid
    done
else
	rm -rf mats
	mv mats_short mats
fi
matlab -nodisplay -nosplash -r "gen_plot($LONG, $DT); exit(0)"
matlab -nodesktop -nosplash -r "gen_video($LONG, $DT, $FPS); exit(0)"
mv mats mats_short
mv imgs imgs_short
mkdir imgs
mkdir mats

mv mats_long mats
mv mats_short mats
mv imgs_long imgs
mv imgs_short imgs

for img in "imgs/*/*.png"
	do mogrify -trim +repage $img
done

for img in "imgs/*/*.avi"
    do convert -quiet -delay 1 $img +map $img.gif
done