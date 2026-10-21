#!/usr/bin/env python3
"""Manage the per-run log folders written by scripts/run_front.sh."""

import argparse
import json
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


def dir_size(path):
    total = 0
    for base, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(base) / name).stat().st_size
            except OSError:
                pass
    return total


def human_size(size):
    size = float(size)
    for unit in ('B', 'KB', 'MB'):
        if size < 1024:
            return f'{size:.0f} {unit}' if unit == 'B' else f'{size:.1f} {unit}'
        size /= 1024
    return f'{size:.1f} GB'


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def describe_run(run):
    meta = read_json(run / 'metadata.json')
    config = read_json(run / 'launch_config.json')
    status = meta.get('exit_status')
    return {
        'name': run.name,
        'size': dir_size(run),
        'exit': 'running/unknown' if status is None else str(status),
        'mode': config.get('mode', '-'),
        'input': config.get('input', '-'),
        'git': meta.get('git_revision', '-'),
    }


def format_runs(root, runs):
    rows = [describe_run(run) for run in runs]
    lines = [f'{"RUN":<21}{"SIZE":>10}  {"EXIT":<16}{"MODE":<14}{"INPUT":<7}GIT']
    for row in rows:
        lines.append(
            f'{row["name"]:<21}{human_size(row["size"]):>10}  {row["exit"]:<16}'
            f'{row["mode"]:<14}{row["input"]:<7}{row["git"]}')
    total = sum(row['size'] for row in rows)
    lines.append(f'{len(rows)} run(s), {human_size(total)} total in {root}')
    return '\n'.join(lines)

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
    commands.add_parser('list', help='list previous runs and disk usage')
    cleanup = commands.add_parser('prune', help='delete old run folders')
    cleanup.add_argument('--keep-runs', type=int, default=env_int('VISION_LOG_KEEP_RUNS', 30))
    cleanup.add_argument('--keep-days', type=int, default=env_int('VISION_LOG_KEEP_DAYS', 14))
    cleanup.add_argument('--protect', help='run folder that must never be deleted')
    cleanup.add_argument('--dry-run', action='store_true', help='show what would be deleted')
    args = parser.parse_args(argv)
    root = log_root()
    if args.command == 'list':
        print(format_runs(root, list_runs(root)))
    elif args.command == 'prune':
        removed = prune(root, args.keep_runs, args.keep_days, args.protect, args.dry_run)
        verb = 'would remove' if args.dry_run else 'removed'
        print(f'[vision_logs] {verb} {len(removed)} old run(s) from {root}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
