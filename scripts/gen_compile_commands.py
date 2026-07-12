#!/usr/bin/env python3
"""
Regenerate compile_commands.json for VSCode IntelliSense.

Two problems with just running `compiledb make <target>` directly:

1. This Makefile hides real compiler command lines behind short
   "Compiling X.c" messages unless invoked with V=1 - compiledb can only
   record commands it can actually see, so anything compiled while V=1
   isn't set silently gets no entry (or a stale one from whatever previous
   run happened to have V=1 set). Always clean + build with V=1 here so
   every file gets recorded in one pass.

2. Board hardware files (e.g. hwconf/example/vesc_h753/hw_h753_core.c) are
   never compiled as their own translation unit - hwconf/hw.c does
   `#include HW_SOURCE`, which textually splices the board file's content
   into hw.c's compilation. GCC never invokes on hw_h753_core.c by name,
   so no amount of regenerating compile_commands.json will ever produce a
   native entry for it - that's what VSCode's "not found in
   compile_commands.json" warning is about for these files. The fix is to
   synthesize an entry for them by cloning whichever entry actually
   #included them, found via the .dep/*.d files GCC already writes
   (-MD -MP -MF), rather than trying to get GCC to compile them directly.

Usage:
    python scripts/gen_compile_commands.py [board]

Defaults to the h753 board if none given.
"""

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CC_JSON = ROOT / "compile_commands.json"
DEP_DIR = ROOT / ".dep"


def run(cmd, **kwargs):
    print(f"$ {' '.join(cmd)}")
    subprocess.run(cmd, cwd=ROOT, check=True, **kwargs)


def dep_file_for_entry(entry):
    """Find the -MF argument (the .d file) in a compile_commands.json entry."""
    args = entry["arguments"]
    for i, a in enumerate(args):
        if a == "-MF" and i + 1 < len(args):
            return (ROOT / args[i + 1]).resolve()
    return None


def parse_dep_c_files(dep_path):
    """Return the set of .c files listed as prerequisites in a GCC .d file."""
    if not dep_path.exists():
        return set()
    text = dep_path.read_text(errors="ignore")
    # Drop the "target:" prefix and line-continuation backslashes, then split on whitespace.
    text = text.split(":", 1)[-1].replace("\\\n", " ")
    return {tok for tok in text.split() if tok.endswith(".c")}


def clone_entry_for_file(entry, new_file_abs):
    """Clone a compile_commands.json entry, pointing it at a different source file."""
    new_entry = json.loads(json.dumps(entry))  # deep copy
    args = new_entry["arguments"]
    old_file = entry["file"]
    # The compiled source file appears as a bare positional argument, e.g.
    # ["...", "hwconf/hw.c", "-o", "build/.../hw.o"]. Swap it for the new file.
    for i, a in enumerate(args):
        if a == old_file:
            args[i] = str(new_file_abs)
            break
    new_entry["file"] = str(new_file_abs)
    return new_entry


def main():
    board = sys.argv[1] if len(sys.argv) > 1 else "h753"

    run(["make", f"{board}_clean"])
    run(["compiledb", "-f", "make", "V=1", "-j16", board])

    with open(CC_JSON) as f:
        entries = json.load(f)

    known_files = {str((ROOT / e["file"]).resolve()) for e in entries}
    synthesized = []

    for entry in entries:
        dep_path = dep_file_for_entry(entry)
        if dep_path is None:
            continue
        for dep_c in parse_dep_c_files(dep_path):
            dep_c_abs = (ROOT / dep_c).resolve() if not Path(dep_c).is_absolute() else Path(dep_c)
            key = str(dep_c_abs)
            if key in known_files:
                continue
            known_files.add(key)
            synthesized.append(clone_entry_for_file(entry, dep_c_abs))
            print(f"  + synthesized entry for {dep_c} (via {entry['file']})")

    if synthesized:
        entries.extend(synthesized)
        with open(CC_JSON, "w") as f:
            json.dump(entries, f, indent=2)
        print(f"Added {len(synthesized)} synthesized entries. Total: {len(entries)}")
    else:
        print("No #include-only .c files found needing a synthesized entry.")


if __name__ == "__main__":
    main()
