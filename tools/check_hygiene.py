#!/usr/bin/env python3
"""Public-release hygiene gate for this package / repository.

Checks, over every ``.py .md .tex .json .jsonl .csv .txt .ps1 .bat .cff .yml
.yaml .sh .cfg .toml .ini`` file:

  1. absolute Windows or POSIX home paths (a drive letter, a Users directory,
     or a /home prefix);
  2. names of folders that no longer exist in this package;
  3. editor / crash leftovers (``.tmp``, ``.swp``, ``.swo``, ``.bak``, ``~$*``,
     ``.#*``, ``.xXXXXXX``, ``.DS_Store``, ``Thumbs.db``, ``__pycache__``,
     ``.pyc``) and stray dotfiles.

Repo mode (the default) is GitHub-compatible: ``.git/`` is skipped entirely,
and ``.gitignore``, ``.gitattributes`` and ``.github/`` are legitimate
repository metadata, not artefacts.  Package mode (``--package-mode``) keeps
the old strict behaviour and flags every dotfile.

It prints a per-extension file count and a hit count, and exits non-zero if
anything is found.  The search literals are assembled from fragments so that
this file does not match itself.

    python tools/check_hygiene.py                # repo mode (default)
    python tools/check_hygiene.py --repo-mode    # explicit
    python tools/check_hygiene.py --package-mode # strict, no dotfile allowlist
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXTS = {
    ".py", ".md", ".tex", ".json", ".jsonl", ".csv", ".txt", ".ps1", ".bat",
    ".cff", ".yml", ".yaml", ".sh", ".cfg", ".toml", ".ini",
}

# assembled from fragments on purpose: keeps this file out of its own results
DRIVE = re.compile("[" + "CD" + "]:[\\\\/]")
USERS = re.compile("Use" + "rs[\\\\/]")
HOME = re.compile("(?:^|[\\s(\"'`=])/ho" + "me/")

STALE = [
    "02_Main" + "_V2_Experiment",
    "03_Perce" + "ption_Robustness",
    "04_Onl" + "ine_Warning_and_Timing",
    "05_Qui" + "ck_Access",
    "01_Fin" + "al_Manuscript",
    "05_" + "\u4ee3\u7801",
    "07_" + "\u590d\u73b0\u5305",
]

# legitimate repository metadata in repo mode
ALLOWED_DOTFILES = {".gitignore", ".gitattributes"}
ALLOWED_DOTDIRS = {".github"}
SKIP_DIRS = {".git"}

TEMP_SUFFIXES = (".tmp", ".temp", ".swp", ".swo", ".bak", ".orig", ".rej", ".pyc")
TEMP_PREFIXES = ("~$", ".#")
TEMP_NAMES = {".DS_Store", "Thumbs.db", "desktop.ini", "__pycache__"}
TEMP_RE = re.compile(r"\.x[A-Za-z0-9]{6}$")
EDITOR_LOCK_RE = re.compile(r"^(?:\.~lock\..+#|#.+#)$")


def is_temp_name(name: str) -> bool:
    return (
        name in TEMP_NAMES
        or name.endswith(TEMP_SUFFIXES)
        or name.startswith(TEMP_PREFIXES)
        or bool(TEMP_RE.search(name))
        or bool(EDITOR_LOCK_RE.match(name))
    )


def is_allowed_dotfile(name: str, repo_mode: bool) -> bool:
    return repo_mode and name in ALLOWED_DOTFILES


def is_allowed_dotdir(name: str, repo_mode: bool) -> bool:
    return repo_mode and name in ALLOWED_DOTDIRS


def scan(repo_mode: bool):
    counts = {}
    hits = []
    temp = []

    for dirpath, dirnames, filenames in os.walk(ROOT):
        keep = []
        for name in dirnames:
            if name in SKIP_DIRS:
                continue  # .git/ is not part of the deliverable at all
            if name.startswith("."):
                if is_allowed_dotdir(name, repo_mode):
                    keep.append(name)
                else:
                    temp.append(
                        os.path.relpath(os.path.join(dirpath, name), ROOT).replace(os.sep, "/") + "/"
                    )
                continue
            if name == "__pycache__":
                temp.append(
                    os.path.relpath(os.path.join(dirpath, name), ROOT).replace(os.sep, "/") + "/"
                )
                continue
            keep.append(name)
        dirnames[:] = keep

        for name in filenames:
            path = Path(dirpath) / name
            if name in SKIP_DIRS:
                continue  # .git may also be a file (worktrees / submodules)
            ext = path.suffix.lower()
            if ext in EXTS:
                counts[ext] = counts.get(ext, 0) + 1
                try:
                    text = path.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    text = ""
                rel = path.relative_to(ROOT).as_posix()
                if text:
                    for number, line in enumerate(text.splitlines(), 1):
                        if DRIVE.search(line) or USERS.search(line) or HOME.search(line):
                            hits.append((rel, number, "absolute path", line.strip()[:130]))
                        for stale in STALE:
                            if stale in line:
                                hits.append((rel, number, "stale folder name", line.strip()[:130]))
            if name.startswith(".") and not is_allowed_dotfile(name, repo_mode):
                temp.append(path.relative_to(ROOT).as_posix())
            elif is_temp_name(name):
                temp.append(path.relative_to(ROOT).as_posix())

    return counts, hits, temp


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--repo-mode",
        dest="repo_mode",
        action="store_true",
        default=True,
        help="GitHub-compatible mode (default): skip .git/, allow .gitignore, "
             ".gitattributes and .github/.",
    )
    parser.add_argument(
        "--package-mode",
        dest="repo_mode",
        action="store_false",
        help="strict mode: flag every dotfile, including .gitignore.",
    )
    args = parser.parse_args(argv)

    counts, hits, temp = scan(args.repo_mode)

    print("mode:", "repo" if args.repo_mode else "package")
    print("scanned extensions (files):")
    for ext in sorted(counts):
        print(f"   {ext:<8s} {counts[ext]}")
    print(f"   {'total':<8s} {sum(counts.values())}")

    print("\nabsolute-path / stale-name hits:", len(hits))
    for rel, number, what, line in hits[:40]:
        print(f"   {rel}:{number}  [{what}] {line}")
    if len(hits) > 40:
        print(f"   ... {len(hits) - 40} more")

    print("dotfiles / temp artefacts:", len(temp))
    for item in temp:
        print("   ", item)

    ok = not hits and not temp
    print("\nHYGIENE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
