#!/bin/bash
source ./common.sh
load_module

UUID1=$(create_mdev)
UUID2=$(create_mdev)
UUID3=$(create_mdev)

run_writer $UUID1 VM1 5000000 100
run_writer $UUID2 VM2 5000000 100
run_writer $UUID3 VM3 5000000 100

wait_writers

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t2.log

python3 parse_mdev_log.py $LOG --csv results/csv/t2.csv \
    > results/summary/t2.txt
