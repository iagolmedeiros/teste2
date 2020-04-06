#!/bin/bash

if [ $1 == "c" ]
then
    rm Dl* Ul* evalvid* *pcap output *txt *xml > /dev/null
elif [ $1 == "r" ]
then
    ./waf --run lte > output 2>&1
elif [ $1 == "n" ]
then 
    ../netanim-3.108/NetAnim
fi
