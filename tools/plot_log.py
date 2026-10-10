#!/usr/bin/env python3
"""Host-side dashboard for the AEB CSV log (optional; NOT run on the QNX Pi).
   usage: python3 plot_log.py aeb_log.csv   -> aeb_timeline.png
   Shows the collision timeline: range & safe distance, TTC, closing speed,
   measured acceleration, AEB state and decision->actuation latency."""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

STATES = ["NORMAL", "WARNING", "PARTIAL_BRAKE", "STRONG_BRAKE", "EMERGENCY_BRAKE",
          "STOPPED", "RADAR_FAULT", "IMU_FAULT", "MOTOR_FAULT", "SYSTEM_FAULT"]
path = sys.argv[1] if len(sys.argv) > 1 else "aeb_log.csv"
t, rng, dsf, ttc, vc, acc, st, lat, ev = [], [], [], [], [], [], [], [], []
with open(path) as f:
    for r in csv.DictReader(f):
        if r["kind"] == "EVENT":
            ev.append((float(r["t_ms"]) / 1000, r["event"]))
            continue
        t.append(float(r["t_ms"]) / 1000)
        rng.append(float(r["range_m"]) if r["target_valid"] == "1" else float("nan"))
        dsf.append(min(float(r["d_safe_full_m"]), 5.0))
        x = float(r["ttc_s"]); ttc.append(x if 0 <= x < 10 else float("nan"))
        vc.append(float(r["v_close_mps"])); acc.append(float(r["a_long_mps2"]))
        st.append(STATES.index(r["state"]) if r["state"] in STATES else -1)
        lat.append(float(r["cmd_latency_us"]))
fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
ax[0].plot(t, rng, label="radar range (in-path target)"); ax[0].plot(t, dsf, "--", label="d_safe FULL")
ax[0].set_ylabel("m"); ax[0].legend(); ax[0].set_title("AEB collision timeline")
ax[1].plot(t, ttc, color="tab:red"); ax[1].set_ylabel("TTC s")
ax[2].plot(t, vc, label="closing speed"); ax[2].plot(t, acc, label="IMU forward accel")
ax[2].set_ylabel("m/s, m/s2"); ax[2].legend()
ax[3].step(t, st, where="post"); ax[3].set_yticks(range(len(STATES))); ax[3].set_yticklabels(STATES, fontsize=7)
ax[4].plot(t, lat, ".", ms=2); ax[4].set_ylabel("decision->GPIO us"); ax[4].set_xlabel("time s")
for tt, e in ev:
    if "STATE" in e or "EMERGENCY" in e or "FAULT" in e:
        for a in ax: a.axvline(tt, color="grey", lw=0.5, alpha=0.5)
plt.tight_layout(); plt.savefig("aeb_timeline.png", dpi=110)
print("written aeb_timeline.png")
