#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select camera port number, default is 0"
	exit 1
fi

echo "set camera port to" "$1"
sed -i "s/camport: .*/camport: $1/g" ../moniarm_cv/param/cvparam.yaml
