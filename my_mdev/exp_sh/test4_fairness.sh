#!/bin/bash
source ./common.sh
load_module

UUID=$(create_mdev)

run_writer $UUID VM1 5000 100
run_writer $UUID VM2 5000 100
run_writer $UUID VM3 5000 100

wait_writers

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t4.log

python3 parse_mdev_log.py $LOG --detail \
    > results/summary/t4_raw.txt

python3 calc_jain.py results/summary/t4_raw.txt \
    > results/summary/t4_jain.txt

cleanup
