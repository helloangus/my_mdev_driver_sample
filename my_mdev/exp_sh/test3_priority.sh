#!/bin/bash
source ./common.sh
load_module

UUID_H=$(create_mdev)
UUID_M=$(create_mdev)
UUID_L=$(create_mdev)

echo 10 | sudo tee /sys/bus/mdev/devices/$UUID_H/priority
echo 5 | sudo tee /sys/bus/mdev/devices/$UUID_M/priority
echo 1  | sudo tee /sys/bus/mdev/devices/$UUID_L/priority

run_writer $UUID_H HIGH 10000 8
run_writer $UUID_L LOW  10000 40

sleep 0.23
run_writer $UUID_M MEDIUM 10000 8

wait_writers

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t3.log

python3 parse_mdev_log.py $LOG --detail \
    > results/summary/t3.txt

cleanup