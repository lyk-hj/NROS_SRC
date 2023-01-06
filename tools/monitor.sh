#!/bin/sh
/home/nvidia/Desktop/real/build/run
export LD_LIBRARY_PATH=/home/nvidia/Desktop/real/build/run
while true; do
        server=ps aux | grep CenterServer_d | grep -v grep
        if [ ! "$server" ]; then
            /home/nvidia/Desktop/real/build/run 
	    echo "restart"
            sleep 10
        fi
        sleep 5
done
