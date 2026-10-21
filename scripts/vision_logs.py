#!/usr/bin/env python3
"""Manage the per-run log folders written by scripts/run_front.sh."""

import argparse
import os
import re
import shutil
import sys
import time
from pathlib import Path

RUN_NAME = re.compile(r'^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$')


def log_root():
    return Path(os.environ.get('VISION_LOG_ROOT') or Path.home() / '.ros/robosub/logs')


def list_runs(root):
    """Run folders, newest first. Only folders named like a run timestamp count."""
    if not root.is_dir():
        return []
    return sorted(
        (path for path in root.iterdir() if path.is_dir() and RUN_NAME.match(path.name)),
        reverse=True)


def runs_to_prune(root, keep_runs, keep_days, protect=None, now=None):
    """Runs beyond the newest keep_runs, or older than keep_days. 0 disables a limit."""
    now = time.time() if now is None else now
    protected = Path(protect).resolve() if protect else None
    doomed = []
    for index, run in enumerate(list_runs(root), start=1):
        if protected is not None and run.resolve() == protected:
            continue
        too_many = keep_runs > 0 and index > keep_runs
        too_old = keep_days > 0 and now - run.stat().st_mtime > keep_days * 86400
        if too_many or too_old:
            doomed.append(run)
    return doomed


def prune(root, keep_runs, keep_days, protect=None, dry_run=False, now=None):
    doomed = runs_to_prune(root, keep_runs, keep_days, protect, now)
    if not dry_run:
        for run in doomed:
            shutil.rmtree(run)
    return doomed


def env_int(name, default):
    try:
        return max(int(os.environ.get(name, default)), 0)
    except ValueError:
        return default


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    cleanup = commands.add_parser('prune', help='delete old run folders')
    cleanup.add_argument('--keep-runs', type=int, default=env_int('VISION_LOG_KEEP_RUNS', 30))
    cleanup.add_argument('--keep-days', type=int, default=env_int('VISION_LOG_KEEP_DAYS', 14))
    cleanup.add_argument('--protect', help='run folder that must never be deleted')
    cleanup.add_argument('--dry-run', action='store_true', help='show what would be deleted')
    args = parser.parse_args(argv)
    if args.command == 'prune':
        root = log_root()
        removed = prune(root, args.keep_runs, args.keep_days, args.protect, args.dry_run)
        verb = 'would remove' if args.dry_run else 'removed'
        print(f'[vision_logs] {verb} {len(removed)} old run(s) from {root}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
