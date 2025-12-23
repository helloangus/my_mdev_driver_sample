#!/bin/bash
set -e

MOD=host_mdev_sched
SYSFS_BASE=/sys/devices/platform/mdev_dsched/mdev_supported_types/mdev_dsched-mdev_dsched
LOG_DIR=/var/log
RESULT_DIR=$(pwd)/results

mkdir -p $RESULT_DIR/{raw,csv,summary}

load_module() {
    sudo modprobe vfio vfio_iommu_type1 mdev
    sudo insmod ../host_mdev_sched.ko || true
}

cleanup() {
    sudo rmmod host_mdev_sched || true
}

create_mdev() {
    UUID=$(uuidgen)
    echo $UUID | sudo tee $SYSFS_BASE/create > /dev/null
    echo $UUID
}


WRITER_PIDS=()
run_writer() {
    UUID=$1
    ID=$2
    SIZE=64
    INTERVAL=$3
    COUNT=$4

    sudo ../mdev_writer \
    --uuid $UUID \
    --id $ID \
    --size $SIZE \
    --interval $INTERVAL \
    --count $COUNT &


    PID=$!
    WRITER_PIDS+=($PID)
}

wait_writers() {
    for pid in "${WRITER_PIDS[@]}"; do
        wait $pid
    done
    WRITER_PIDS=()
}
