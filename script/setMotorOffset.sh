#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 M0_OFF M1_OFF M2_OFF M3_OFF"
    echo "example: $0 0 0 3 5"
	exit 1
fi

echo "set MOTOR_ID" "$1" "$2" "$3" "$4"

cd ../arduino/
for d in ./*/; do
	cd "$d";
	for filename in ./*.ino; do
		if [[ $(awk '/M0_ID/' "$filename") ]]; then
			echo 'Matched' "$filename"
			sed -i "s/#define ANGLE_0OFF.*/#define ANGLE_0OFF $1/g" "$filename"
			sed -i "s/#define ANGLE_1OFF.*/#define ANGLE_1OFF $2/g" "$filename"
			sed -i "s/#define ANGLE_2OFF.*/#define ANGLE_2OFF $3/g" "$filename"
			sed -i "s/#define ANGLE_3OFF.*/#define ANGLE_3OFF $4/g" "$filename"
			sed -i "s/#define DUAL_SHOULDER.*/#define DUAL_SHOULDER 0/g" "$filename"
		fi
	done
	cd ..
done