import argparse
import csv
import math
import socket
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import NamedTuple
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

GCM_NONCE_LEN = 12
GCM_TAG_LEN   = 16
AES_KEY_LEN   = 32


ERASE_LINE  = "\r\033[2K"
CURSOR_UP   = "\033[1A"
CURSOR_DOWN = "\033[1B"



def load_key(path: str) -> bytes:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Key file not found: {path}")
    data = p.read_bytes()
    if len(data) != AES_KEY_LEN:
        raise ValueError(f"Key must be exactly {AES_KEY_LEN} bytes, got {len(data)}")
    return data



def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except TimeoutError:
            continue
        if not chunk:
            raise ConnectionError("Connection closed by remote host")
        buf.extend(chunk)
    return bytes(buf)


def decrypt_frame(frame_bytes: bytes, aesgcm: AESGCM) -> bytes:
    if len(frame_bytes) < GCM_NONCE_LEN + GCM_TAG_LEN:
        raise ValueError("Frame too short to contain nonce + tag")
    nonce      = frame_bytes[:GCM_NONCE_LEN]
    tag        = frame_bytes[GCM_NONCE_LEN : GCM_NONCE_LEN + GCM_TAG_LEN]
    ciphertext = frame_bytes[GCM_NONCE_LEN + GCM_TAG_LEN:]
    return aesgcm.decrypt(nonce, ciphertext + tag, None)



def format_bitrate(bps: float) -> str:
    if bps >= 1_000_000:
        return f"{bps / 1_000_000:.2f} Mbps"
    if bps >= 1_000:
        return f"{bps / 1_000:.2f} Kbps"
    return f"{bps:.2f} bps"


class EstimatorResult(NamedTuple):
    p_max:   float  # probability of most likely symbol sequence
    h_min:   float  # min-entropy in bits/byte
    longest: int    # longest tuple length (v) or SEQ_LEN for Markov


def estimate_markov(data: bytes) -> EstimatorResult:

    n = len(data)
    if n < 2:
        return EstimatorResult(p_max=1.0, h_min=1.0, longest=0)

    SEQ_LEN = 128
    n_bits  = n * 8
    n_words = n // 8

    # transition_count indexed by 2-bit key: 0b00, 0b01, 0b10, 0b11
    tc   = [0, 0, 0, 0]
    ones = 0

    for w in range(n_words):
        # little-endian matches C++ reinterpret_cast<const uint64_t*>
        word = int.from_bytes(data[w * 8 : (w + 1) * 8], 'little')
        ones += bin(word).count('1')
        for _ in range(32):          
            tc[word & 0b11] += 1
            word >>= 2

    P1 = ones / n_bits
    P0 = 1.0 - P1

    t0  = tc[0b00] + tc[0b01]
    t1  = tc[0b10] + tc[0b11]
    P00 = tc[0b00] / t0 if t0 else 0.5;  P01 = 1.0 - P00
    P10 = tc[0b10] / t1 if t1 else 0.5;  P11 = 1.0 - P10
    tp  = [P00, P01, P10, P11]           # tp[0b00]=P00, tp[0b01]=P01, …

    def p_s(b: int) -> float:
        """Max path probability for a SEQ_LEN-step sequence starting in state b."""
        c_b = b ^ 0b01                   # complement bit: 0b00↔0b01, 0b11↔0b10
        return max(
            tp[b]   ** (SEQ_LEN - 1),                                        # stays:      xx…x
            tp[c_b] ** (SEQ_LEN // 2) * tp[c_b ^ 0b11] ** (SEQ_LEN // 2 - 1),  # alternates: xyxy…
            tp[c_b] * tp[b ^ 0b11] ** (SEQ_LEN - 2),                        # flips once: xy…y
        )

    p_hat = max(P0 * p_s(0b00), P1 * p_s(0b11))

    # C++ divides by SEQ_LEN to get bits/bit; ×8 converts to bits/byte
    h_min = min(-math.log2(p_hat) / SEQ_LEN , 1.0) if p_hat > 0 else 1.0

    return EstimatorResult(p_max=p_hat, h_min=h_min, longest=SEQ_LEN)




def estimate_lrs(data: bytes) -> EstimatorResult | None:

    n       = len(data)
    n_words = n // 8
    if n_words == 0:
        return None

    # Unpack to flat bit array — LSB-first within each 64-bit word (little-endian),
    # same ordering the C++ Markov uses with reinterpret_cast<const uint64_t*>
    bits = bytearray(n_words * 64)
    for w in range(n_words):
        word = int.from_bytes(data[w * 8 : (w + 1) * 8], 'little')
        base = w * 64
        for i in range(64):
            bits[base + i] = word & 1
            word >>= 1

    n_bits  = len(bits)   # = n_words * 64
    CUTOFF  = 35
    u       = 0
    v       = 0
    found_u = False
    p_hat   = 0.0
    mask    = 0           # filled in once t is known: (1 << t) - 1

    t = 1
    while True:
        limit = n_bits - t + 1
        if limit <= 0:
            break

        mask = (1 << t) - 1

        # Seed the first window key: bit[0] at position 0, bit[t-1] at position t-1
        key = 0
        for j in range(t):
            key |= bits[j] << j

        histogram: dict[int, int] = defaultdict(int)
        histogram[key] += 1
        max_count = 1

        # Slide: drop LSB (bit[i-1]), shift right, insert new MSB (bit[i+t-1])
        for i in range(1, limit):
            key = (key >> 1) | (bits[i + t - 1] << (t - 1))
            histogram[key] += 1
            c = histogram[key]
            if c > max_count:
                max_count = c

        # Step 1: find u = first t where max_count < CUTOFF
        if not found_u:
            if max_count < CUTOFF:
                u       = t
                found_u = True
            t += 1
            continue                   # always skip P_W calculation for t ≤ u

        # Steps 2–3: for t > u, accumulate P_W and track v
        if max_count >= 2:
            v = t

            # numerator   = Σ C(count, 2)  for all tuples with count ≥ 2
            numerator   = sum(c * (c - 1) // 2 for c in histogram.values() if c >= 2)
            # denominator = C(limit, 2)
            denominator = (limit * (limit - 1)) / 2.0

            if denominator > 0.0:
                P_W     = numerator / denominator
                P_max_W = P_W ** (1.0 / t)
                if P_max_W > p_hat:
                    p_hat = P_max_W
        else:
            break                      # no tuple repeats at this length — done

        t += 1

    if not found_u or p_hat <= 0.0:
        return None

    # Step 4: upper-confidence bound (use n_bits as sample count)
    p_u   = min(p_hat + 2.576 * math.sqrt(p_hat * (1.0 - p_hat) / (n_bits - 1)), 1.0)
    # Step 5: min-entropy in bits/bit, matching Markov scale
    h_min = min(-math.log2(p_u), 1.0)

    return EstimatorResult(p_max=p_hat, h_min=h_min, longest=v)



class MetricsLogger:

    FIELDS = ["timestamp", "elapsed_s", "bps",
              "h_markov", "h_lrs", "h_min",
              "frames", "total_bits"]

    def __init__(self, path: str):
        self._path   = Path(path)
        self._file   = self._path.open("a", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        # Write header only if the file is new / empty
        if self._path.stat().st_size == 0:
            self._writer.writeheader()
            self._file.flush()

    def log(self, *, elapsed_s: float, bps: float,
            h_markov: float, h_lrs: float,
            frames: int, total_bits: int) -> None:
        self._writer.writerow({
            "timestamp":  datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "elapsed_s":  f"{elapsed_s:.3f}",
            "bps":        f"{bps:.2f}",
            "h_markov":   f"{h_markov:.6f}",
            "h_lrs":      f"{h_lrs:.6f}",
            "h_min":      f"{min(h_markov, h_lrs):.6f}",
            "frames":     frames,
            "total_bits": total_bits,
        })
        self._file.flush()

    def close(self) -> None:
        self._file.close()



class Display:
    def __init__(self, show_hashes: bool, show_bitrate: bool, show_entropy: bool):
        self.show_hashes  = show_hashes
        self.show_bitrate = show_bitrate
        self.show_entropy = show_entropy

        # How many fixed lines are reserved at the bottom
        self._fixed = int(show_entropy) + int(show_bitrate or show_hashes)

        # Seed the fixed lines so cursor positions are stable from the start
        if show_entropy:
            sys.stdout.write("[Markov:--  LRS:--  Min:-- b/b]\n")
        if show_bitrate or show_hashes:
            sys.stdout.write("\n")
        sys.stdout.flush()

    # Move cursor to the entropy line (top fixed line) and erase it
    def _goto_entropy(self):
        lines_up = 2 if (self.show_bitrate or self.show_hashes) else 1
        sys.stdout.write(CURSOR_UP * lines_up + ERASE_LINE)

    # Move cursor to the status line (bottom fixed line) and erase it
    def _goto_status(self):
        sys.stdout.write(CURSOR_UP + ERASE_LINE)

    def update_entropy(self, h_markov: float, h_lrs: float):
        if not self.show_entropy:
            return
        h_min = min(h_markov, h_lrs)
        line  = f"[Markov:{h_markov:.3f}  LRS:{h_lrs:.3f}  Min:{h_min:.3f} b/b]"
        if self.show_bitrate or self.show_hashes:
            # entropy is the line above status — go up 2, write, come back down
            sys.stdout.write(CURSOR_UP * 2 + ERASE_LINE + line + "\n" + CURSOR_DOWN)
        else:
            self._goto_entropy()
            sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def update_status(self, bps: float, plaintext: bytes | None):
        if not (self.show_bitrate or self.show_hashes):
            return
        parts = []
        if self.show_bitrate:
            parts.append(f"[{format_bitrate(bps)}]")
        if self.show_hashes and plaintext is not None:
            parts.append(plaintext.hex())
        self._goto_status()
        sys.stdout.write("  ".join(parts) + "\n")
        sys.stdout.flush()

    def final_summary(self, received: int, total_bits: int, elapsed: float,
                      h_markov: float | None, h_lrs: float | None):
        # Move past all fixed lines so the summary appears below them
        if self._fixed == 0:
            sys.stdout.write("\n")
        bps = total_bits / elapsed if elapsed > 0 else 0.0
        print(f"\n[INFO] {received} frame(s), {total_bits} bits "
              f"in {elapsed:.1f}s = {format_bitrate(bps)}")
        if self.show_entropy and h_markov is not None and h_lrs is not None:
            h_min = min(h_markov, h_lrs)
            print(f"[INFO] Last entropy — Markov:{h_markov:.4f}  "
                  f"LRS:{h_lrs:.4f}  Min:{h_min:.4f} bits/byte")



def receive_loop(host: str, port: int, aesgcm: AESGCM,
                 output_file=None,
                 show_hashes:   bool = False,
                 show_bitrate:  bool = False,
                 show_entropy:  bool = False,
                 entropy_every: int  = 1000,
                 count:         int  = 0,
                 metrics_log:   MetricsLogger | None = None):

    print(f"Connecting to {host}:{port} ...", flush=True)

    with socket.create_connection((host, port)) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(1.0)
        print("Connected.\n", flush=True)

        display     = Display(show_hashes, show_bitrate, show_entropy)
        received    = 0
        total_bits  = 0
        start_time  = time.monotonic()
        entropy_buf = bytearray()
        h_markov: float | None = None
        h_lrs:    float | None = None

        try:
            while True:
                try:
                    raw_len = recv_exact(sock, 4)
                except TimeoutError:
                    continue

                frame_len = struct.unpack("!I", raw_len)[0]
                if frame_len < GCM_NONCE_LEN + GCM_TAG_LEN:
                    continue

                frame_bytes = recv_exact(sock, frame_len)
                try:
                    plaintext = decrypt_frame(frame_bytes, aesgcm)
                except Exception as e:
                    print(f"[ERROR] Decryption failed: {e}", file=sys.stderr)
                    continue

                received   += 1
                total_bits += len(plaintext) * 8

                if output_file:
                    output_file.write(plaintext)
                    output_file.flush()

                if show_entropy or metrics_log:
                    entropy_buf.extend(plaintext)
                    if len(entropy_buf) >= entropy_every:
                        buf      = bytes(entropy_buf)
                        r_markov = estimate_markov(buf)
                        r_lrs    = estimate_lrs(buf)
                        h_markov = r_markov.h_min
                        # LRS returns None when data lacks repeated tuples; fall back to Markov
                        h_lrs    = r_lrs.h_min if r_lrs is not None else h_markov
                        elapsed  = time.monotonic() - start_time
                        bps      = total_bits / elapsed if elapsed > 0 else 0.0
                        if show_entropy:
                            display.update_entropy(h_markov, h_lrs)
                        if metrics_log:
                            metrics_log.log(
                                elapsed_s  = elapsed,
                                bps        = bps,
                                h_markov   = h_markov,
                                h_lrs      = h_lrs,
                                frames     = received,
                                total_bits = total_bits,
                            )
                        entropy_buf.clear()

                elapsed = time.monotonic() - start_time
                bps     = total_bits / elapsed if elapsed > 0 else 0.0
                display.update_status(bps, plaintext)

                if count and received >= count:
                    print(f"\nReached requested count of {count} frames. Done.")
                    break

        except ConnectionError as e:
            print(f"\n[INFO] Connection closed: {e}")
        except KeyboardInterrupt:
            elapsed = time.monotonic() - start_time
            display.final_summary(received, total_bits, elapsed, h_markov, h_lrs)



def parse_args():
    p = argparse.ArgumentParser(
        description="Receive and decrypt AES-256-GCM encrypted TRNG hashes over TCP."
    )
    p.add_argument("--host",          default="127.0.0.1",
                   help="Server IP address (default: 127.0.0.1)")
    p.add_argument("--port",          type=int, default=7777,
                   help="Server TCP port (default: 7777)")
    p.add_argument("--key",           default="./key",
                   help="Path to 32-byte binary AES key file (default: ./key)")
    p.add_argument("--out",           default=None,
                   help="File path to append raw decrypted bytes to")
    p.add_argument("--show-hashes",   action="store_true",
                   help="Show latest decrypted hash in the status line")
    p.add_argument("--bitrate",       action="store_true",
                   help="Show live cumulative bitrate in the status line")
    p.add_argument("--entropy",       action="store_true",
                   help="Show NIST SP 800-90B Markov + LRS estimates above status")
    p.add_argument("--entropy-every", type=int, default=1000,
                   help="Re-run estimators every N bytes (default: 1000)")
    p.add_argument("--count",         type=int, default=0,
                   help="Stop after N frames (default: 0 = run forever)")
    p.add_argument("--metrics-log",   default=None,
                   help="CSV file to append per-sample metrics to (timestamp, bps, h_markov, h_lrs, …)")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        key = load_key(args.key)
    except (FileNotFoundError, ValueError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

    aesgcm = AESGCM(key)
    print(f"Key loaded from '{args.key}' ({AES_KEY_LEN} bytes).")

    out_f = None
    if args.out:
        out_f = open(args.out, "ab")
        print(f"Appending raw bytes to '{args.out}'.")

  

    # sizes = [int (1000 * pow(1.2,n)) for n in range(50)] 
    # for n in sizes:
    #     data = bytes(open("log.bits", "rb").read(n))
        
    #     t0 = time.perf_counter()
    #     h_markov = estimate_markov(data).h_min
    #     t_markov = time.perf_counter() - t0
        
    #     t0 = time.perf_counter()
    #     h_lrs = estimate_lrs(data).h_min
    #     t_lrs = time.perf_counter() - t0
        
    #     print(f"{n}, {t_markov:.4f}, {t_lrs:.4f}, {h_markov:.4f}, {h_lrs:.4f}")

    metrics = None
    if args.metrics_log:
        metrics = MetricsLogger(args.metrics_log)
        print(f"Logging metrics to '{args.metrics_log}'.")

    try:
        receive_loop(
            host          = args.host,
            port          = args.port,
            aesgcm        = aesgcm,
            output_file   = out_f,
            show_hashes   = args.show_hashes,
            show_bitrate  = args.bitrate,
            show_entropy  = args.entropy,
            entropy_every = args.entropy_every,
            count         = args.count,
            metrics_log   = metrics,
        )
    finally:
        if out_f:
            out_f.close()
        if metrics:
            metrics.close()


if __name__ == "__main__":
    main()
