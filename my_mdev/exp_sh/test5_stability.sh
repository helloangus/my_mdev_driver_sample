#!/bin/bash
source ./common.sh
load_module

UUID=$(create_mdev)

for i in $(seq 1 5); do
    run_writer $UUID VM1 5000 100
    wait_writers
done

LOG=$(ls -t /var/log/mdev_dsched_*.txt | head -1)
cp $LOG results/raw/t5.log

python3 parse_mdev_log.py $LOG --csv results/csv/t5.csv \
    > results/summary/t5.txt

cleanup