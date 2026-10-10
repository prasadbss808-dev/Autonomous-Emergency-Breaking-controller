#!/bin/sh
# Run all 12 AEB test scenarios in real-time simulation (no hardware moves).
cd "$(dirname "$0")"
mkdir -p sim_logs
pass=0; fail=0
for n in 1 2 3 4 5 6 7 8 9 10 11 12; do
    ./aeb -m sim -s $n -q -l sim_logs/sim_$n.csv < /dev/null > sim_logs/sim_$n.txt 2>&1
    if [ $? -eq 0 ]; then pass=$((pass+1)); r=PASS; else fail=$((fail+1)); r=FAIL; fi
    echo "scenario $n: $r   $(grep 'SIM TEST' sim_logs/sim_$n.txt | tail -1)"
done
echo "== $pass passed, $fail failed (logs in ./sim_logs) =="
