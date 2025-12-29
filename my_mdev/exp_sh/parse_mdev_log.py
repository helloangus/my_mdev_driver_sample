#!/usr/bin/env python3
import re
import sys
import argparse
from collections import defaultdict

# 匹配块头：[uuid]
UUID_RE = re.compile(
    r'^\[(?P<uuid>[^\]]+)\]\s*$'
)

# 匹配数据行：id=... seq=... ts=...
LOG_RE = re.compile(
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

    current_uuid = None

    with open(args.logfile) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            # 1. 是否是 UUID 块头
            m_uuid = UUID_RE.match(line)
            if m_uuid:
                current_uuid = m_uuid.group("uuid")
                continue

            # 2. 是否是数据行
            m = LOG_RE.search(line)
            if not m:
                continue

            # 3. 没有 UUID 上下文，直接忽略（防御性）
            if current_uuid is None:
                continue

            wid = m.group("id")
            ts  = float(m.group("ts"))

            s = stats[current_uuid]
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
