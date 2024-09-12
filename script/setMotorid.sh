#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 M0_ID M1_ID M2_ID M3_ID"
    echo "example: setMotorid.sh 1 2 3 4"
	exit 1
fi

echo "set MOTOR_ID" "$1" "$2" "$3" "$4"

cd ../arduino/
for d in ./*/; do
	cd "$d";
	for filename in ./*.ino; do
		if [[ $(awk '/M0_ID/' "$filename") ]]; then
			echo 'Matched' "$filename"
			sed -i "s/#define M0_ID.*/#define M0_ID $1/g" "$filename"
			sed -i "s/#define M1_ID.*/#define M1_ID $2/g" "$filename"
			sed -i "s/#define M2_ID.*/#define M2_ID $3/g" "$filename"
			sed -i "s/#define M3_ID.*/#define M3_ID $4/g" "$filename"
			sed -i "s/#define M1M_ID.*/#define M1M_ID 0/g" "$filename"
			sed -i "s/#define DUAL_SHOULDER.*/#define DUAL_SHOULDER 0/g" "$filename"
		fi
	done
	cd ..
done
