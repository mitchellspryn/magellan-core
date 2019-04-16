#!/bin/bash


usage()
{
  echo -e "Usage: ./conedetectorbag2vid.sh (flags) <BagName>";
  echo -e "\tBagName: The name of the bag with the images in it.";
  echo -e "\tFlags:";
  echo -e "\t-n: Will not delete the intermediate video feeds.";
  echo -e "\t-o: Output file name. Defaults to 'output.mp4' if not set"
  exit 1;
}

SHOULD_SAVE_VIDEOS="false"
OUTPUT_FILE_NAME="output.mp4"

while getopts "no:" o; do
    case "${o}" in
        n)
            SHOULD_SAVE_VIDEOS="true"
            ;;
        o)
            OUTPUT_FILE_NAME=${OPTARG}
            ;;
    esac
done
shift $((OPTIND-1))

#Bag name should not be null or spaces
if [[ -z "${1// }" ]]; then
  usage
fi

BAG_PATH=$(readlink -e $1)

echo 'Creating original image...'
./bag2vid.sh -o original.mp4 -t /cone_detector_node/debug_original_image_topic ${BAG_PATH}

echo 'Creating prediction image...'
./bag2vid.sh -o predicted.mp4 -t /cone_detector_node/debug_predicted_image_topic ${BAG_PATH}

echo 'Creating processed image...'
./bag2vid.sh -o processed.mp4 -t /cone_detector_node/debug_processed_image_topic ${BAG_PATH}

echo 'Creating debug draw image...'
./bag2vid.sh -o draw.mp4 -t /cone_detector_node/debug_draw_image_topic ${BAG_PATH}

echo 'Creating overlay image...'
ffmpeg -i original.mp4 -i draw.mp4 -i predicted.mp4 -i processed.mp4 -filter_complex "nullsrc=size=640x480 [base]; [0:v] setpts=PTS-STARTPTS, scale=320x240 [upperleft]; [1:v] setpts=PTS-STARTPTS, scale=320x240 [upperright]; [2:v] setpts=PTS-STARTPTS, scale=320x240 [lowerleft]; [3:v] setpts=PTS-STARTPTS, scale=320x240 [lowerright]; [base][upperleft] overlay=shortest=1 [tmp1]; [tmp1][upperright] overlay=shortest=1:x=320 [tmp2]; [tmp2][lowerleft] overlay=shortest=1:y=240 [tmp3]; [tmp3][lowerright] overlay=shortest=1:x=320:y=240" -vcodec libx264 -pix_fmt yuv420p ${OUTPUT_FILE_NAME}

if [[ ${SHOULD_SAVE_VIDEOS} != "true" ]]; then
    echo 'Cleaning up...'
    rm original.mp4
    rm predicted.mp4
    rm processed.mp4
    rm draw.mp4
fi
