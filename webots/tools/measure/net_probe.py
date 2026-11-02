"""Measure the Pi <-> laptop Wi-Fi link with the simulator's message sizes.

Laptop (server):   python net_probe.py server
Pi (client):       python3 net_probe.py client --host <laptop IP> --count 200

UDP: the client sends each message size `count` times at `rate` per second;
the server echoes it back. Reports round-trip time (median / p95), jitter and
loss per size. Sizes default to the simulator's messages:
  40 ACK/STATE/CLAIM, 160 report header, 1800 thumb report, 4360 crop report,
  1500 fleet status.
TCP: one bulk transfer of a full 1280x720 JPEG-sized payload (138 KB) for
throughput.

Results go to net_probe.csv (one row per packet) on the client. On Windows
allow the server through the firewall (UDP and TCP on --port) when asked.
"""
import argparse
import csv
import socket
import statistics
import struct
import threading
import time

HDR = struct.Struct("!IId")          # size, seq, send time
DEFAULT_SIZES = "40,160,1500,1800,4360"
BULK_BYTES = 138_000


def serve(port):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", port))

    def tcp_loop():
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(1)
        while True:
            conn, _ = srv.accept()
            with conn:
                head = b""
                while len(head) < 4:
                    head += conn.recv(4 - len(head))
                n = struct.unpack("!I", head)[0]
                got = 0
                while got < n:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    got += len(chunk)
                conn.sendall(struct.pack("!I", got))

    threading.Thread(target=tcp_loop, daemon=True).start()
    print(f"echo server on UDP/TCP port {port}; Ctrl+C to stop")
    while True:
        data, addr = udp.recvfrom(65535)
        udp.sendto(data, addr)


def client(host, port, sizes, count, rate, out):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.settimeout(1.0)
    rows = []
    for size in sizes:
        rtts, lost = [], 0
        for seq in range(count):
            payload = HDR.pack(size, seq, time.perf_counter()).ljust(size, b"\0")
            udp.sendto(payload, (host, port))
            try:
                while True:
                    data, _ = udp.recvfrom(65535)
                    s, q, t0 = HDR.unpack_from(data)
                    if s == size and q == seq:
                        rtt = (time.perf_counter() - t0) * 1000
                        rtts.append(rtt)
                        rows.append({"proto": "udp", "size": size, "seq": seq, "rtt_ms": round(rtt, 3), "lost": 0})
                        break
            except socket.timeout:
                lost += 1
                rows.append({"proto": "udp", "size": size, "seq": seq, "rtt_ms": "", "lost": 1})
            time.sleep(1 / rate)
        if rtts:
            rtts.sort()
            jitter = statistics.mean(abs(a - b) for a, b in zip(rtts, rtts[1:])) if len(rtts) > 1 else 0
            print(f"UDP {size:>6} B: rtt median {statistics.median(rtts):6.1f} ms  "
                  f"p95 {rtts[int(0.95 * (len(rtts) - 1))]:6.1f} ms  jitter {jitter:5.1f} ms  "
                  f"loss {100 * lost / count:4.1f}%")
        else:
            print(f"UDP {size:>6} B: all {count} lost")

    tcp = socket.create_connection((host, port), timeout=10)
    t0 = time.perf_counter()
    tcp.sendall(struct.pack("!I", BULK_BYTES) + b"\0" * BULK_BYTES)
    got = struct.unpack("!I", tcp.recv(4))[0]
    dt = time.perf_counter() - t0
    tcp.close()
    print(f"TCP {BULK_BYTES} B: {dt * 1000:.0f} ms -> {8 * got / dt / 1e6:.2f} Mbit/s")
    rows.append({"proto": "tcp", "size": BULK_BYTES, "seq": 0, "rtt_ms": round(dt * 1000, 3), "lost": 0})

    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["server", "client"])
    ap.add_argument("--host", help="laptop IP (client mode)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--sizes", default=DEFAULT_SIZES)
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--rate", type=float, default=20.0, help="packets per second")
    ap.add_argument("--out", default="net_probe.csv")
    args = ap.parse_args()
    if args.mode == "server":
        serve(args.port)
    else:
        if not args.host:
            ap.error("--host is required in client mode")
        client(args.host, args.port, [int(s) for s in args.sizes.split(",")], args.count, args.rate, args.out)


if __name__ == "__main__":
    main()
