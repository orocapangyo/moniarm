#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one between these"
    echo "csicam, usbcam"
	exit 1
fi

cd ../moniarm_control/launch
if [ "$1" == "usbcam" ]; then
    for filename in ./*.launch.py; do
        if [[ $(awk '/csicam/' "$filename") ]]; then
            echo 'Matched' "$filename"
            sed -i "s/csicam/usbcam/g" "$filename"
        fi
    done
else
    for filename in ./*.launch.py; do
        if [[ $(awk '/usbcam/' "$filename") ]]; then
            echo 'Matched' "$filename"
            sed -i "s/usbcam/csicam/g" "$filename"
        fi
    done
fi
