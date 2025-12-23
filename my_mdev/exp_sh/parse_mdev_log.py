#!/usr/bin/env python3
import re
import sys
import argparse
from collections import defaultdict

LOG_RE = re.compile(
    r'\[(?P<uuid>[^\]]+)\]\s+'
    r'id=(?P<id>\S+)\s+'
    r'seq=(?P<seq>\d+)\s+'
    r'ts=(?P<ts>\d+\.\d+)'
)

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile", help="mdev_dsched log file")
    ap.add_argument("--csv", help="export per-uuid stats to csv")
    ap.add_argument("--detail", action="store_true",
                    help="print per-writer detail")
    return ap.parse_args()

def main():
    args = parse_args()

    stats = defaultdict(lambda: {
        "count": 0,
        "first_ts": None,
        "last_ts": None,
        "writers": defaultdict(int),
    })

    with open(args.logfile) as f:
        for line in f:
            m = LOG_RE.search(line)
            if not m:
                continue

            uuid = m.group("uuid")
            wid  = m.group("id")
            ts   = float(m.group("ts"))

            s = stats[uuid]
            s["count"] += 1
            s["writers"][wid] += 1

            if s["first_ts"] is None:
                s["first_ts"] = ts
            s["last_ts"] = ts

    print("==== mdev scheduling statistics ====")
    for uuid, s in stats.items():
        duration = (s["last_ts"] - s["first_ts"]) if s["count"] > 1 else 0
        rate = s["count"] / duration if duration > 0 else 0

        print(f"\nUUID: {uuid}")
        print(f"  messages : {s['count']}")
        print(f"  duration : {duration:.6f} s")
        print(f"  msg/s    : {rate:.2f}")

        if args.detail:
            print("  writers:")
            for wid, cnt in s["writers"].items():
                print(f"    {wid:10s} {cnt:8d}")

    if args.csv:
        with open(args.csv, "w") as out:
            out.write("uuid,count,duration,msg_per_sec\n")
            for uuid, s in stats.items():
                duration = (s["last_ts"] - s["first_ts"]) if s["count"] > 1 else 0
                rate = s["count"] / duration if duration > 0 else 0
                out.write(f"{uuid},{s['count']},{duration:.6f},{rate:.2f}\n")

if __name__ == "__main__":
    main()
