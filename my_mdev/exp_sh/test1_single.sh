#!/bin/bash
source ./common.sh
load_module

UUID=$(create_mdev)
run_writer $UUID VM1 10000 100
wait_writers

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t1.log

python3 parse_mdev_log.py $LOG --csv results/csv/t1.csv \
    > results/summary/t1.txt
    
cleanup