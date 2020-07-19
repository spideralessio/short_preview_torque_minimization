LONG=1
DT=0.001
FPS=60
USE_CACHE=1

if [ $USE_CACHE -eq 1 ]; then
	rm -rf imgs/*
	mv mats/mats_long mats_long
	mv mats/mats_short mats_short
	rm -rf mats
fi

if [ $USE_CACHE -eq 0 ]; then
    echo "MTN"
    matlab -nodisplay -nosplash -r "MTN($LONG, $DT); exit(0)"
    echo "MTND"
    matlab -nodisplay -nosplash -r "MTND($LONG, $DT); exit(0)"
    echo "MBP"
    matlab -nodisplay -nosplash -r "MBP($LONG, $DT); exit(0)"
    echo "MBPD"
    matlab -nodisplay -nosplash -r "MBPD($LONG, $DT); exit(0)"
    echo "MTIWN"
    matlab -nodisplay -nosplash -r "MTIWN($LONG, $DT); exit(0)"
    echo "MTIWND"
    matlab -nodisplay -nosplash -r "MTIWND($LONG, $DT); exit(0)"
    echo "MBIWP"
    matlab -nodisplay -nosplash -r "MBIWP($LONG, $DT); exit(0)"
    echo "MBIWPD"
    matlab -nodisplay -nosplash -r "MBIWPD($LONG, $DT); exit(0)"
    echo "MTSIWN"
    matlab -nodisplay -nosplash -r "MTSIWN($LONG, $DT); exit(0)"
    echo "MTSIWND"
    matlab -nodisplay -nosplash -r "MTSIWND($LONG, $DT); exit(0)"
    echo "MBSIWP"
    matlab -nodisplay -nosplash -r "MBSIWP($LONG, $DT); exit(0)"
    echo "MBSIWPD"
    matlab -nodisplay -nosplash -r "MBSIWPD($LONG, $DT); exit(0)"
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
if [ $USE_CACHE -eq 0 ]; then
    echo "MTN"
    matlab -nodisplay -nosplash -r "MTN($LONG, $DT); exit(0)"
    echo "MTND"
    matlab -nodisplay -nosplash -r "MTND($LONG, $DT); exit(0)"
    echo "MBP"
    matlab -nodisplay -nosplash -r "MBP($LONG, $DT); exit(0)"
    echo "MBPD"
    matlab -nodisplay -nosplash -r "MBPD($LONG, $DT); exit(0)"
    echo "MTIWN"
    matlab -nodisplay -nosplash -r "MTIWN($LONG, $DT); exit(0)"
    echo "MTIWND"
    matlab -nodisplay -nosplash -r "MTIWND($LONG, $DT); exit(0)"
    echo "MBIWP"
    matlab -nodisplay -nosplash -r "MBIWP($LONG, $DT); exit(0)"
    echo "MBIWPD"
    matlab -nodisplay -nosplash -r "MBIWPD($LONG, $DT); exit(0)"
    echo "MTSIWN"
    matlab -nodisplay -nosplash -r "MTSIWN($LONG, $DT); exit(0)"
    echo "MTSIWND"
    matlab -nodisplay -nosplash -r "MTSIWND($LONG, $DT); exit(0)"
    echo "MBSIWP"
    matlab -nodisplay -nosplash -r "MBSIWP($LONG, $DT); exit(0)"
    echo "MBSIWPD"
    matlab -nodisplay -nosplash -r "MBSIWPD($LONG, $DT); exit(0)"
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