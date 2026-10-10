#!/bin/sh
# perf_capture.sh - records a performance capture of the running AEB into a perf_<date> folder
# usage: ./perf_capture.sh [seconds(20)] [noload|load]  - trace, thread list, CPU use, logs, report
cd "$(dirname "$0")"
SEC=${1:-20}
MODE=${2:-noload}
TS=$(date +%Y%m%d_%H%M%S)
OUT=perf_$TS
NCPU=4
# returns true if a process with this name is running
running() { pidin -p "$1" 2>/dev/null | grep -q "$1"; }
if ! running aeb; then
    echo "perf_capture: aeb is not running. Start it first:  ./master.sh bg   (or ./aeb_master in another shell)"
    exit 1
fi
[ "$(id -u)" = "0" ] || echo "perf_capture: WARNING - not root; tracelogger needs root (use: su -c ./perf_capture.sh)"
mkdir -p $OUT
echo "perf_capture: writing everything to $PWD/$OUT"
# system info: QNX version and CPUs
{
    echo "== uname -a"; uname -a
    echo; echo "== pidin info"; pidin info
} > $OUT/system.txt 2>&1
# RTOS: Priority Preemption - save every thread with its priority and policy (FIFO / RR)
for p in aeb aeb_guard aeb_dash aeb_vision; do
    if running $p; then
        echo "===== $p : threads (prio + policy f=FIFO r=RR, state) ====="
        pidin -p $p
        echo
    fi
done > $OUT/pidin_threads.txt 2>&1
# runmasks show which CPU core each aeb thread is pinned to
for p in aeb; do
    running $p && { echo "===== $p : runmasks ====="; pidin -p $p rmasks; echo; }
done > $OUT/pidin_runmasks.txt 2>&1
# load mode: NCPU busy loops at priority 5, to show the AEB threads still preempt them
LOAD_PIDS=""
if [ "$MODE" = "load" ]; then
    echo "perf_capture: starting CPU load: $NCPU busy loops at priority 5 (below every AEB thread)"
    i=0
    while [ $i -lt $NCPU ]; do
        on -p 5 sh -c 'while :; do :; done' &
        LOAD_PIDS="$LOAD_PIDS $!"
        i=$((i + 1))
    done
fi
echo "perf_capture: System Profiler trace for $SEC s -> $OUT/aeb_trace.kev"
# kernel trace (tracelogger) for SEC seconds and CPU use per process (hogs), both in background
tracelogger -n 0 -s $SEC -w -f $OUT/aeb_trace.kev > $OUT/tracelogger.txt 2>&1 &
TL=$!
hogs -i $((SEC / 2)) -s 2 > $OUT/hogs.txt 2>&1 &
HG=$!
echo
echo "  >>> NOW: put an obstacle (hand / box) in front of the radar, remove it, repeat 2-3 times <<<"
echo
# countdown while the trace is recording
t=0
while [ $t -lt $SEC ]; do
    printf "  %2d s left\r" $((SEC - t))
    sleep 1
    t=$((t + 1))
done
echo
# wait for the trace to finish, then stop hogs and the load loops
wait $TL 2>/dev/null
kill $HG 2>/dev/null
[ -n "$LOAD_PIDS" ] && kill $LOAD_PIDS 2>/dev/null
# save the slog2 system logs and a copy of the AEB CSV log
slog2info -b aeb > $OUT/slog2_aeb.txt 2>&1
slog2info 2>/dev/null | tail -n 400 > $OUT/slog2_system_tail.txt
[ -f aeb_log.csv ] && cp aeb_log.csv $OUT/
# Log: Response Latency - make the latency / jitter summary from the CSV with perf_report.sh
if [ -f $OUT/aeb_log.csv ]; then
    PERIOD_MS=20 ./perf_report.sh $OUT/aeb_log.csv > $OUT/perf_summary.txt 2>&1
    echo "MODE: $MODE (load = $NCPU busy loops at prio 5 during the trace)" >> $OUT/perf_summary.txt
fi
# show the result files and the summary, then how to open the trace in Momentics
echo
ls -l $OUT
echo
[ -f $OUT/perf_summary.txt ] && cat $OUT/perf_summary.txt
[ -s $OUT/aeb_trace.kev ] || echo "perf_capture: WARNING - no trace file; see $OUT/tracelogger.txt"
echo
echo "Copy to your laptop:  scp -r qnxuser@<pi-ip>:$PWD/$OUT ."
echo "Then in Momentics: File > Open File... > $OUT/aeb_trace.kev  (opens the System Profiler)"
