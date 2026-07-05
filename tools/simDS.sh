#!/bin/bash
# My first shell script
cd /home/gabriel_wang/work/Kuavo/cmake-build-debug/tools
echo "Hello $USER"
i=0
while true; do
 i=$((i + 1))
 sleep .05
 echo -e "\n\nrun unit test  $i th times\n\n"
 mosquitto_pub -t "test/topic1" -f hello.bin
 sleep 1
 mosquitto_pub -t "test/topic1" -f button_pressed.bin
 sleep 1
 mosquitto_pub -t "test/topic1" -f button_released.bin
 sleep 1
done
