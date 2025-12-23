#!/bin/bash
source ./common.sh
load_module

UUID_H=$(create_mdev)
UUID_L=$(create_mdev)

echo 10 | sudo tee /sys/bus/mdev/devices/$UUID_H/priority
echo 1  | sudo tee /sys/bus/mdev/devices/$UUID_L/priority

run_writer $UUID_H HIGH 5000000 100
run_writer $UUID_L LOW  5000000 100

wait_writers

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t3.log

python3 parse_mdev_log.py $LOG --detail \
    > results/summary/t3.txt
