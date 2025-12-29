#!/bin/bash
set -e

for t in test1_single.sh \
         test2_multi.sh \
         test3_priority.sh \
         test4_fairness.sh \
         test5_stability.sh
do
    echo "==== Running $t ===="
    bash $t
done

echo "ALL TESTS COMPLETED"
