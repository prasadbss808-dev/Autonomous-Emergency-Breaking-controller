#!/bin/sh
# perf_report.sh - reads the AEB CSV log and prints loop timing, latency and deadline numbers
# usage: ./perf_report.sh [aeb_log.csv]   (PERIOD_MS and DEADLINE_US can be changed from the env)
CSV=${1:-aeb_log.csv}
PERIOD_MS=${PERIOD_MS:-20}
# Timer: Braking Deadline - 50 ms
DEADLINE_US=${DEADLINE_US:-50000}
[ -f "$CSV" ] || { echo "perf_report: $CSV not found"; exit 1; }
# temp files are deleted when the script ends
TMP=${TMPDIR:-/tmp}/perf_report.$$
trap 'rm -f $TMP.*' EXIT INT TERM
# find the cmd_latency_us column from the CSV header (column 18 if not found)
LATCOL=$(head -n 1 "$CSV" | tr ',' '\n' | grep -n '^cmd_latency_us$' | cut -d: -f1)
[ -n "$LATCOL" ] || LATCOL=18
# Log: Response Latency - sorted latency of every cycle, and of every emergency brake event
awk -F, -v C="$LATCOL" '$2=="CYCLE" && $C+0 > 0 { print $C+0 }' "$CSV" | sort -n > $TMP.lat
grep "EMERGENCY BRAKE applied" "$CSV" | sed 's/.*decision->outputs \([0-9][0-9]*\) us.*/\1/' | grep '^[0-9][0-9]*$' | sort -n > $TMP.emerg
echo "================ AEB PERFORMANCE REPORT ================"
echo "source           : $CSV"
echo "control period   : $PERIOD_MS ms (kernel timer pulse)   braking deadline: $DEADLINE_US us"
echo
# RTOS: Deadline Scheduling - control loop period, jitter (difference from P) and overruns
awk -F, -v P="$PERIOD_MS" '
$2=="CYCLE" {
    n++
    if (n == 1) { t0 = $1 }
    if (n > 1) {
        d = $1 - prev
        if (n == 2 || d < dmin) dmin = d
        if (n == 2 || d > dmax) dmax = d
        j = (d - P) * 1000; if (j < 0) j = -j
        jsum += j; if (j > jmax) jmax = j
        if (d > P * 1.5) over++
    }
    prev = $1
    st[$3]++
}
END {
    if (n < 2) { print "not enough CYCLE rows"; exit }
    printf "CONTROL LOOP (prediction thread, FIFO 58)\n"
    printf "  cycles             : %d over %.1f s\n", n, (prev - t0) / 1000
    printf "  period min/avg/max : %.3f / %.3f / %.3f ms\n", dmin, (prev - t0) / (n - 1), dmax
    printf "  jitter avg / max   : %.0f us / %.0f us\n", jsum / (n - 1), jmax
    printf "  overruns (>1.5xP)  : %d\n", over + 0
    printf "  cycles per state   :"
    for (s in st) printf " %s=%d", s, st[s]
    printf "\n\n"
}' "$CSV"
# RTOS: Message Passing - min / avg / p50 / p99 / max of the MsgSend/MsgReply latency
N=$(wc -l < $TMP.lat | tr -d ' ')
if [ "$N" -gt 0 ]; then
    awk -v N="$N" '{ v[NR] = $1; s += $1 }
    END {
        p50 = v[int((N - 1) * 0.50) + 1]; p99 = v[int((N - 1) * 0.99) + 1]
        printf "DECISION -> MOTOR COMMAND (MsgSend/MsgReply to brake thread, every cycle)\n"
        printf "  samples            : %d\n", N
        printf "  min/avg/p50/p99/max: %.0f / %.1f / %.0f / %.0f / %.0f us\n\n", v[1], s / N, p50, p99, v[N]
    }' $TMP.lat
fi
# RTOS: Priority Preemption - emergency brake latency and how many missed the 50 ms deadline
E=$(wc -l < $TMP.emerg | tr -d ' ')
echo "EMERGENCY BRAKE (priority-63 pulse -> brake thread -> GPIO)"
if [ "$E" -gt 0 ]; then
    awk -v N="$E" -v D="$DEADLINE_US" '{ v[NR] = $1; s += $1; if ($1 > D) miss++ }
    END {
        printf "  events             : %d\n", N
        printf "  min/avg/max        : %.0f / %.1f / %.0f us\n", v[1], s / N, v[N]
        printf "  deadline misses    : %d of %d\n\n", miss + 0, N
    }' $TMP.emerg
else
    echo "  events             : 0  (no emergency brake in this log - put an obstacle in front of the radar)"
    echo
fi
# last 15 safety events (watchdog, deadline missed, faults) with their time in seconds
echo "SAFETY EVENTS"
grep -E "SAFE-LATCH|WATCHDOG|DEADLINE MISSED|RADAR_FAULT|radar not fresh|SYSTEM_FAULT" "$CSV" |
    awk -F, '{ t = $1; sub(/^[^"]*"/, ""); sub(/"$/, ""); printf "  %10.3f s  %s\n", t / 1000, $0 }' | tail -n 15
[ -n "$(grep -E 'SAFE-LATCH|WATCHDOG|DEADLINE MISSED|RADAR_FAULT|radar not fresh|SYSTEM_FAULT' "$CSV")" ] || echo "  none"
echo
# short summary lines to copy onto the presentation slide
echo "---------------- VALUES FOR THE PERFORMANCE SLIDE ----------------"
if [ "$E" -gt 0 ]; then
    awk -v N="$E" -v D="$DEADLINE_US" '{ v[NR] = $1; s += $1; if ($1 > D) miss++ }
    END { printf "Emergency brake latency (detection -> GPIO): %.0f us avg / %.0f us max (%d events)\n", s / N, v[N], N
          printf "Braking deadline (50 ms) misses: %d of %d brakes\n", miss + 0, N }' $TMP.emerg
fi
awk -F, -v P="$PERIOD_MS" '$2=="CYCLE" { n++; if (n > 1) { j = ($1 - prev - P) * 1000; if (j < 0) j = -j; if (j > m) m = j } prev = $1 }
    END { if (n > 1) printf "Control loop: %d ms period, max jitter +-%.0f us over %d cycles\n", P, m, n }' "$CSV"
