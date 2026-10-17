#!/usr/bin/env python3
"""Extract the MATLAB Function / Stateflow chart code embedded in a .slx model.

Each MATLAB Function block in a Simulink model is stored in the .slx zip as a
Stateflow chart (simulink/stateflow/chart_*.xml) whose state script holds the
function body.  This tool writes those bodies out as plain .m text so the
algorithm is readable (and diffable) outside MATLAB.

Usage:
    python tools/extract_model_code.py MODEL.slx OUTDIR [--name PREFIX]

The output is verbatim: no reformatting, no edits.  Files are named
<PREFIX>_<n>_<function>.m, where <function> is taken from the MATLAB signature.
"""
import argparse
import html
import os
import re
import sys
import zipfile


def chart_scripts(slx_path):
    """Return [(chart_entry, function_name, code)] for every script in the model."""
    out = []
    with zipfile.ZipFile(slx_path) as z:
        names = [n for n in z.namelist()
                 if n.startswith("simulink/stateflow/chart_") and n.endswith(".xml")]
        for entry in sorted(names, key=lambda s: int(re.search(r"(\d+)", os.path.basename(s)).group(1))):
            xml = z.read(entry).decode("utf8", "ignore")
            for script in re.findall(r'<P Name="script">(.*?)</P>', xml, re.S):
                code = html.unescape(script)
                m = re.search(r"function\s+(?:\[[^\]]*\]|[\w]+)\s*=\s*([\w]+)", code)
                out.append((entry, m.group(1) if m else "anonymous", code))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model")
    ap.add_argument("outdir")
    ap.add_argument("--name", default=None,
                    help="filename prefix (default: model basename)")
    args = ap.parse_args(argv)

    prefix = args.name or os.path.splitext(os.path.basename(args.model))[0]
    os.makedirs(args.outdir, exist_ok=True)
    scripts = chart_scripts(args.model)
    if not scripts:
        print(f"no embedded MATLAB code found in {args.model}", file=sys.stderr)
        return 1

    index = []
    for i, (entry, fn, code) in enumerate(scripts, start=1):
        fname = f"{prefix}_{i:02d}_{fn}.m"
        with open(os.path.join(args.outdir, fname), "w", encoding="utf-8") as fh:
            fh.write(f"% Extracted verbatim from {os.path.basename(args.model)} "
                     f"({entry}) by tools/extract_model_code.py\n")
            fh.write(code.rstrip() + "\n")
        index.append((fname, fn, len(code.splitlines())))

    for fname, fn, nlines in index:
        print(f"{fname:<48} {fn:<22} {nlines:>4} lines")
    print(f"\n{len(index)} function block(s) written to {args.outdir}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
