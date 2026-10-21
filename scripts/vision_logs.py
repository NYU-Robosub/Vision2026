#!/usr/bin/env python3
"""Manage the per-run log folders written by scripts/run_front.sh."""

import argparse
import datetime
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path
from collections import deque

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


def tail_lines(path, count):
    """Last `count` lines of a log, plus the byte offset where following should start."""
    with open(path, errors='replace') as handle:
        lines = list(deque(handle, maxlen=count)) if count > 0 else []
    return lines, os.path.getsize(path)


def follow(path, offset, poll=0.5):
    """Yield complete lines appended after `offset`, waiting for more."""
    with open(path, 'rb') as handle:
        handle.seek(offset)
        while True:
            position = handle.tell()
            line = handle.readline()
            if line.endswith(b'\n'):
                yield line.decode(errors='replace')
            else:
                handle.seek(position)  # Partial line: wait until it is finished.
                time.sleep(poll)


LEVELS = {'DEBUG': 10, 'INFO': 20, 'WARN': 30, 'ERROR': 40, 'FATAL': 50}
LOG_LINE = re.compile(
    r'^\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*(?:\[\d+(?:\.\d+)?\]\s*)?\[([^\]]+)\]')


def parse_level(name):
    name = name.upper()
    return 'WARN' if name == 'WARNING' else name


def classify(lines):
    """(level, node) for each line. Lines without a prefix, such as the 'Check:'
    list under an error, belong to the record above them."""
    level, node, result = None, None, []
    for line in lines:
        match = LOG_LINE.match(line)
        if match:
            level, node = parse_level(match.group(1)), match.group(2)
        result.append((level, node))
    return result


def search_lines(lines, text=None, level=None, min_level=None, node=None, context=0,
                 times=None, since=None, until=None):
    """Lines matching every filter plus `context` lines around each: (number, line, matched)."""
    needle = text.lower() if text else None
    hits = []
    for index, (line, (line_level, line_node)) in enumerate(zip(lines, classify(lines))):
        if level and line_level != level:
            continue
        if min_level and (line_level is None or LEVELS[line_level] < LEVELS[min_level]):
            continue
        if node and (line_node is None or node.lower() not in line_node.lower()):
            continue
        if needle and needle not in line.lower():
            continue
        if times is not None:
            if since is not None and times[index] < since:
                continue
            if until is not None and times[index] > until:
                continue
        hits.append(index)
    shown = {}
    for index in hits:
        for near in range(max(0, index - context), min(len(lines), index + context + 1)):
            shown[near] = shown.get(near, False) or near == index
    return [(near + 1, lines[near].rstrip('\n'), shown[near]) for near in sorted(shown)]


def format_search(run_name, results):
    out, previous = [], None
    for number, line, matched in results:
        if previous is not None and number != previous + 1:
            out.append('--')
        out.append(f'{run_name}:{number}{":" if matched else "-"} {line}')
        previous = number
    return out
 
 
LINE_TIME = re.compile(
    r'^\[(?:DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[(\d+(?:\.\d+)?)\]')


def parse_time(value, now=None):
    """Epoch seconds from '30m', '2h', '1d' (ago) or a UTC time like 2026-10-21T18:41."""
    now = time.time() if now is None else now
    text = value.strip()
    match = re.fullmatch(r'(\d+)([smhd])', text)
    if match:
        unit = {'s': 1, 'm': 60, 'h': 3600, 'd': 86400}[match.group(2)]
        return now - int(match.group(1)) * unit
    for fmt in ('%Y-%m-%dT%H:%M:%S', '%Y-%m-%dT%H:%M', '%Y-%m-%d'):
        try:
            parsed = datetime.datetime.strptime(text.replace(' ', 'T'), fmt)
        except ValueError:
            continue
        return parsed.replace(tzinfo=datetime.timezone.utc).timestamp()
    raise ValueError(
        f"cannot read time '{value}'. Use 30m, 2h, 1d, or a UTC time like 2026-10-21T18:41")


def run_start(run):
    """When a run started (epoch seconds): metadata.json first, then the folder name (UTC)."""
    stamp = read_json(run / 'metadata.json').get('started_utc')
    for value, fmt in ((stamp, '%Y-%m-%dT%H:%M:%SZ'), (run.name, '%Y-%m-%d_%H-%M-%S')):
        if value:
            try:
                parsed = datetime.datetime.strptime(value, fmt)
            except ValueError:
                continue
            return parsed.replace(tzinfo=datetime.timezone.utc).timestamp()
    return run.stat().st_mtime


def line_times(lines, start):
    """Best-known time of each line. Node lines carry their own; others take the last seen."""
    current, result = start, []
    for line in lines:
        match = LINE_TIME.match(line)
        if match:
            current = float(match.group(1))
        result.append(current)
    return result
    

def resolve_run(root, name):
    """Run folder for 'latest', an exact folder name, or a unique prefix."""
    runs = list_runs(root)
    if not runs:
        raise ValueError(f'no runs found in {root}')
    if name == 'latest':
        return runs[0]
    matches = [run for run in runs if run.name == name] or [
        run for run in runs if run.name.startswith(name)]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError(f"no run matches '{name}'. Try: vision_logs.py list")
    raise ValueError(f"'{name}' matches {len(matches)} runs; give a longer name")


def format_run(run):
    meta = read_json(run / 'metadata.json')
    config = read_json(run / 'launch_config.json')
    status = meta.get('exit_status')
    if status is None:
        outcome = 'unknown (still running, or killed before it could record)'
    elif status == 0:
        outcome = '0 (clean exit)'
    else:
        outcome = f'{status} (failed or interrupted)'
    dirty = ' (uncommitted changes)' if meta.get('git_uncommitted_changes') else ''
    lines = [
        f'Run:         {run.name}',
        f'Folder:      {run}',
        f'Started:     {meta.get("started_utc", "-")}',
        f'Finished:    {meta.get("finished_utc", "-")}',
        f'Exit:        {outcome}',
        f'Git:         {meta.get("git_revision", "-")} on {meta.get("git_branch", "-")}{dirty}',
        f'Launch args: {" ".join(meta.get("launch_args", [])) or "(defaults)"}',
    ]
    if config:
        lines.append('Configuration:')
        for key in ('mode', 'input', 'svo_path', 'serial_number', 'database_path',
                    'database_existed', 'zed_config', 'rtabmap_config'):
            if key in config:
                lines.append(f'  {key:<17}{config[key]}')
        for name, topic in sorted(config.get('topics', {}).items()):
            lines.append(f'  {name:<17}{topic}')
    else:
        lines.append('Configuration: not recorded (the launch stopped before it was '
                     'resolved, or the run predates this feature)')
    lines.append(f'Size:        {human_size(dir_size(run))}')
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
    show = commands.add_parser('show', help="show a run's configuration and exit status")
    show.add_argument('run', nargs='?', default='latest',
                      help="'latest' (default), a run folder name, or a unique prefix")
    tail = commands.add_parser('tail', help="print the end of a run's console log")
    tail.add_argument('run', nargs='?', default='latest',
                      help="'latest' (default), a run folder name, or a unique prefix")
    tail.add_argument('-n', '--lines', type=int, default=40, help='lines to show (default 40)')
    tail.add_argument('-f', '--follow', action='store_true',
                      help='keep printing new lines; stop with Ctrl+C')
    search = commands.add_parser('search', help='search the console logs')
    search.add_argument('run', nargs='?', default='latest',
                        help="'latest' (default), a run folder name, or a unique prefix")
    search.add_argument('--all', action='store_true', help='search every run')
    search.add_argument('--text', help='keep lines containing this text (not case sensitive)')
    search.add_argument('--level', type=parse_level, choices=list(LEVELS),
                        help='keep only this level')
    search.add_argument('--min-level', type=parse_level, choices=list(LEVELS),
                        help='keep this level and above')
    search.add_argument('--node', help='keep lines from nodes whose name contains this')
    search.add_argument('--since', help='keep lines at or after this time: 30m, 2h, 1d ago, '
                                        'or a UTC time like 2026-10-21T18:41')
    search.add_argument('--until', help='keep lines at or before this time (same formats)')
    search.add_argument('--context', type=int, default=0, help='lines to show around a match')
    cleanup = commands.add_parser('prune', help='delete old run folders')
    cleanup.add_argument('--keep-runs', type=int, default=env_int('VISION_LOG_KEEP_RUNS', 30))
    cleanup.add_argument('--keep-days', type=int, default=env_int('VISION_LOG_KEEP_DAYS', 14))
    cleanup.add_argument('--protect', help='run folder that must never be deleted')
    cleanup.add_argument('--dry-run', action='store_true', help='show what would be deleted')
    args = parser.parse_args(argv)
    root = log_root()
    if args.command == 'list':
        print(format_runs(root, list_runs(root)))
    elif args.command == 'show':
        try:
            print(format_run(resolve_run(root, args.run)))
        except ValueError as error:
            print(f'[vision_logs] {error}', file=sys.stderr)
            return 1
    elif args.command == 'tail':
        try:
            log = resolve_run(root, args.run) / 'console.log'
            if not log.is_file():
                raise ValueError(f'{log} does not exist')
        except ValueError as error:
            print(f'[vision_logs] {error}', file=sys.stderr)
            return 1
        lines, offset = tail_lines(log, args.lines)
        print(f'[vision_logs] {log}', file=sys.stderr)
        sys.stdout.write(''.join(lines))
        sys.stdout.flush()
        if args.follow:
            try:
                for line in follow(log, offset):
                    sys.stdout.write(line)
                    sys.stdout.flush()
            except KeyboardInterrupt:
                pass
    elif args.command == 'search':
        if not (args.text or args.level or args.min_level or args.node
                or args.since or args.until):
            print('[vision_logs] give at least one of --text, --level, --min-level, --node, '
                  '--since, --until', file=sys.stderr)
            return 1
        try:
            since = parse_time(args.since) if args.since else None
            until = parse_time(args.until) if args.until else None
            runs = list_runs(root) if args.all else [resolve_run(root, args.run)]
        except ValueError as error:
            print(f'[vision_logs] {error}', file=sys.stderr)
            return 1
        total = 0
        for run in runs:
            log = run / 'console.log'
            if not log.is_file():
                continue
            lines = log.read_text(errors='replace').splitlines()
            results = search_lines(
                lines, args.text, args.level, args.min_level, args.node, args.context,
                times=line_times(lines, run_start(run)), since=since, until=until)
            if results:
                total += sum(1 for _, _, matched in results if matched)
                print('\n'.join(format_search(run.name, results)))
        print(f'[vision_logs] {total} matching line(s) in {len(runs)} run(s)', file=sys.stderr)
    elif args.command == 'prune':
        removed = prune(root, args.keep_runs, args.keep_days, args.protect, args.dry_run)
        verb = 'would remove' if args.dry_run else 'removed'
        print(f'[vision_logs] {verb} {len(removed)} old run(s) from {root}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
