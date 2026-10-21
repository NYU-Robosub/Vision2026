# Run logs and troubleshooting

Every run of `scripts/run_front.sh` saves its output in its own folder, so a run can be
examined after it ends, including runs that failed to start.

## Where logs go

```
~/.ros/robosub/logs/2026-10-21_18-38-34/     (the name is the start time, in UTC)
    console.log          everything shown in the terminal, saved as it was printed
    metadata.json        start and finish time, git revision, launch arguments, exit status
    launch_config.json   the resolved configuration: mode, camera serial, database path, topics
    ros/                 ROS launch log files
```

- `launch_config.json` is written once the launch has passed its configuration checks. A run
  that fails those checks (for example a missing database) has no such file. Its
  `metadata.json` still holds the launch arguments and exit status.
- `exit_status` is empty while a run is in progress, or if it was killed before it could
  record one.
- Times are UTC. New York is 4 hours behind UTC during daylight time, 5 hours otherwise.
- The runtime status monitor (`vision_status`) starts next to the launch when it is installed.
  Its output goes into `console.log`.

## Settings (environment variables)

| Variable | Default | Meaning |
|---|---|---|
| `VISION_LOG_ROOT` | `~/.ros/robosub/logs` | where run folders are kept |
| `VISION_LOG_KEEP_RUNS` | `30` | keep at most this many runs (0 turns the limit off) |
| `VISION_LOG_KEEP_DAYS` | `14` | delete runs older than this (0 turns the limit off) |
| `VISION_STATUS_MONITOR` | `1` | set to `0` to not start the runtime status monitor |

Old runs are pruned at the start of each run. The current run is never deleted, and only
folders named like a run timestamp are ever touched.

## Commands

Run from the repository root with `python3 scripts/vision_logs.py <command>`.

```bash
python3 scripts/vision_logs.py list                      # runs, sizes, exit status, mode
python3 scripts/vision_logs.py show latest               # one run's configuration and exit status
python3 scripts/vision_logs.py tail latest --follow      # follow the newest log (Ctrl+C to stop)
python3 scripts/vision_logs.py search --all --level WARN
python3 scripts/vision_logs.py search latest --min-level ERROR
python3 scripts/vision_logs.py search --all --text "data timed out" --context 3
python3 scripts/vision_logs.py search --all --node vision_status --since 2h
python3 scripts/vision_logs.py export latest --output /tmp/vision-run.tar.gz
python3 scripts/vision_logs.py clear --older-than 14d            # preview only
python3 scripts/vision_logs.py clear --older-than 14d --yes      # delete
python3 scripts/vision_logs.py clear --all --yes
python3 scripts/vision_logs.py prune --dry-run
```

A run argument can be `latest`, a full folder name, or a unique prefix such as `2026-10-21_18`.

Notes on searching:

- `--level WARN` also matches lines tagged `[WARNING]`.
- Lines with no `[LEVEL]` tag, such as the `Check:` list under an error, belong to the message
  above them, so `--min-level ERROR` shows a whole error.
- `--since` and `--until` take `30m`, `2h`, `1d` (ago) or a UTC time like `2026-10-21T18:41`.
  Only lines from ROS nodes carry their own timestamp. Other lines use the last time seen above
  them, or the run's start time.
- `clear` shows a preview unless `--yes` is given, and skips a run that has no recorded exit
  status and changed within the last hour, since it may still be running.

## What startup tells you

The launch prints one summary block: live camera or SVO replay, camera serial, mapping or
localization, database path and status, clock source, the main topics, and which node owns
each TF. Risky settings are logged at WARNING level, for example mapping into an existing
database, or a camera serial that differs between the launch argument and the ZED config.

Common failures print what failed and what to check:

| Situation | Where it is reported |
|---|---|
| invalid mode, relative or empty or directory database path | launch error |
| localization with no database | launch error |
| missing or invalid SVO path | launch error |
| `zed_wrapper` not installed | launch error |
| RTAB-Map exits | exit report: info on a clean exit, error with next steps otherwise |
| camera or topic stops publishing | `vision_status` monitor (`NO DATA`, `still waiting`) |

## Not yet verified with hardware

These were tested without a camera, so check them on the robot:

- the startup summary and warnings in a live run
- the RTAB-Map exit report (Ctrl+C once, then once with a bad database)
- Ctrl+C during a live run: `metadata.json` should still get a finish time and exit status,
  and the status monitor should stop
- which serial number the ZED wrapper uses when the launch argument and the YAML disagree
- camera-not-found, missing topics and TF problems at runtime
