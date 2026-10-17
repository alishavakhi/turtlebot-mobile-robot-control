#!/usr/bin/env python3
"""Structural checks on the repository before it is published.

These are cheap, deterministic guards — not a substitute for reading the tree.
Each check prints PASS/FAIL and the script exits non-zero if anything fails.

    python tools/static_checks.py
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEXT_EXT = {".md", ".py", ".m", ".txt", ".csv", ".yaml", ".yml", ".json", ".gitignore", ""}
BINARY_SKIP = {".slx", ".mat", ".pdf", ".png", ".jpg"}

# credential shapes, matched structurally — never by a literal value
CRED_PATTERNS = [
    (r"(?i)\b(api[_-]?key|secret[_-]?key|access[_-]?token|auth[_-]?token|password|passwd)\b\s*[:=]\s*['\"][^'\"]{8,}", "credential assignment"),
    (r"\bghp_[A-Za-z0-9]{20,}", "GitHub token"),
    (r"\bsk-[A-Za-z0-9]{20,}", "OpenAI-style key"),
    (r"\bAKIA[0-9A-Z]{16}\b", "AWS access key"),
    (r"-----BEGIN [A-Z ]*PRIVATE KEY-----", "private key block"),
    (r"\bxox[baprs]-[A-Za-z0-9-]{10,}", "Slack token"),
]
EMAIL = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
PRIVATE_IP = re.compile(r"\b(?:10|192\.168|172\.(?:1[6-9]|2[0-9]|3[01]))\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")
STUDENT_ID = re.compile(r"(?i)\b(matricola|student\s*id)\b")
ROS_MASTER = "ROS_" + "MASTER" + "_URI"   # built at runtime: the checker must not carry the literal
# material that must not be republished
FORBIDDEN_PATHS = [
    r"(?i)\blectur", r"(?i)oral_exam", r"(?i)report template", r"(?i)\bNA\s?[123]\b",
    r"(?i)\bEA\s?[12]\b", r"(?i)johny", r"(?i)filippo", r"(?i)hosam", r"(?i)amir",
    r"(?i)getViconMeasure", r"(?i)bus_conv_fcns", r"(?i)slprj",
]
FORBIDDEN_EXTRA_PDF = "report/Ali_Shavakhi_Final_Report.pdf"


class Checker:
    def __init__(self):
        self.rows = []

    def check(self, name, failures, detail=""):
        self.rows.append((name, bool(not failures), detail or (f"{len(failures)} hit(s)" if failures else "")))
        if failures:
            self.rows[-1] = (name, False, "; ".join(str(f) for f in failures[:5]))

    @property
    def failures(self):
        return [r for r in self.rows if not r[1]]

    def print(self):
        for name, ok, detail in self.rows:
            print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))
        print(f"\n{len(self.rows) - len(self.failures)}/{len(self.rows)} static checks passed")


def iter_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in (".git", "__pycache__")]
        for fn in filenames:
            yield os.path.relpath(os.path.join(dirpath, fn), root)


def read_text(path):
    try:
        with open(path, encoding="utf-8") as fh:
            return fh.read()
    except (UnicodeDecodeError, OSError):
        return ""


def csv_floats():
    vals = []
    for path in glob.glob(os.path.join(ROOT, "data", "metrics", "*.csv")):
        with open(path, encoding="utf-8") as fh:
            for row in csv.reader(fh):
                for cell in row:
                    try:
                        vals.append(float(cell))
                    except ValueError:
                        pass
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    c = Checker()
    files = sorted(iter_files(ROOT))
    text_files = [f for f in files if os.path.splitext(f)[1].lower() in TEXT_EXT]

    # 1 — credentials
    hits = []
    for rel in text_files:
        body = read_text(os.path.join(ROOT, rel))
        for pattern, label in CRED_PATTERNS:
            for m in re.finditer(pattern, body):
                hits.append(f"{rel}: {label}")
    c.check("no credential-shaped strings", hits)

    # 2 — personal data
    emails, ids = [], []
    for rel in text_files:
        body = read_text(os.path.join(ROOT, rel))
        for m in EMAIL.finditer(body):
            if m.group(0) != "rosuser@test.com":
                emails.append(f"{rel}: {m.group(0)}")
        if STUDENT_ID.search(body) and "no student id" not in body.lower():
            ids.append(rel)
    c.check("no email addresses", emails)
    c.check("no student-id field", ids)

    # 3 — lab infrastructure
    ips, uris = [], []
    for rel in text_files:
        body = read_text(os.path.join(ROOT, rel))
        ips += [f"{rel}: {m.group(0)}" for m in PRIVATE_IP.finditer(body)]
        if ROS_MASTER in body:
            uris.append(rel)
    c.check("no private IP addresses", ips)
    c.check("no ROS master URI / lab setup notes", uris)

    # 4 — forbidden material
    bad = [f for f in files if any(re.search(p, f) for p in FORBIDDEN_PATHS)]
    c.check("no course, peer or generated material in the tree", bad)
    pdfs = [f for f in files if f.lower().endswith(".pdf")]
    c.check("the only PDF is the submitted report",
            [f for f in pdfs if f != FORBIDDEN_EXTRA_PDF], f"{len(pdfs)} PDF(s)")

    # 5 — figures referenced by the README must exist and be real PNGs
    readme = read_text(os.path.join(ROOT, "README.md"))
    images = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", readme)
    missing = [i for i in images if not os.path.exists(os.path.join(ROOT, i))]
    c.check("every README image exists", missing, f"{len(images)} embedded image(s)")
    empty = [i for i in images if os.path.getsize(os.path.join(ROOT, i)) < 5000] if not missing else []
    c.check("every README image is a non-trivial PNG", empty)
    c.check("README embeds at least four figures (not links)",
            [] if len(images) >= 4 else [f"only {len(images)}"], f"{len(images)} embedded")

    # 6 — relative links resolve
    broken = []
    for doc in ["README.md"] + sorted(glob.glob(os.path.join("docs", "*.md"))):
        body = read_text(os.path.join(ROOT, doc))
        for link in re.findall(r"(?<!!)\[[^\]]*\]\(([^)#]+)(?:#[^)]*)?\)", body):
            if link.startswith(("http://", "https://", "mailto:")):
                continue
            target = os.path.normpath(os.path.join(os.path.dirname(os.path.join(ROOT, doc)), link))
            if not os.path.exists(target):
                broken.append(f"{doc} -> {link}")
    c.check("every relative link resolves", broken)

    # 7 — no corrupted (0x1A) text survived
    corrupt = [rel for rel in text_files if "\x1a" in read_text(os.path.join(ROOT, rel))]
    c.check("no 0x1A-corrupted text files", corrupt)

    # 8 — every number in a "verified" table must exist in that metric table
    results = read_text(os.path.join(ROOT, "docs", "RESULTS.md"))
    unmatched, checked, blocks = [], 0, 0
    pool = None
    for line in results.splitlines():
        marker = re.match(r"<!--\s*verified:([^\s]+)\s*-->", line.strip())
        if marker:
            path = os.path.join(ROOT, marker.group(1).strip())
            pool = []
            if os.path.exists(path):
                with open(path, encoding="utf-8") as fh:
                    for row in csv.reader(fh):
                        for cell in row:
                            try:
                                pool.append(float(cell))
                            except ValueError:
                                pass
            blocks += 1
            continue
        if line.startswith("<!--") or line.startswith("#"):
            pool = None
            continue
        if pool is None or not line.lstrip().startswith("|"):
            continue
        line = line.replace("\u2212", "-")
        for tok in re.findall(r"(?<![\w.])-?\d*\.\d{3,7}(?![\w.])", line):
            val = float(tok)
            checked += 1
            if not any(abs(val - p) <= max(1e-6, 2e-3 * abs(val)) for p in pool):
                unmatched.append(tok)
    c.check("every decimal in a verified RESULTS.md table traces to its metric table",
            unmatched, f"{checked} values in {blocks} table(s) checked")

    if not args.quiet:
        c.print()
    return 1 if c.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
