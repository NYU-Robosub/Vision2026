# Persistent vision logs: implementation task for Lucy

Status: proposal only. The commands below are the requested interface, not
implemented commands. Keep this PR in draft until implementation and validation
are complete.

## Objective

Replace the startup-summary-only scope of PR #1 with a persistent logging and
troubleshooting workflow. Preserve the useful startup summary and terminal output,
but save enough evidence to diagnose a failed run after its terminal is closed.
Reuse Lucy's summary implementation rather than maintaining two copies. Coordinate
with Sydney's runtime monitor (PR #2); do not reimplement stream-health detection.
The combined work currently exists on `integrate-lucy-sydney` locally. Confirm its
availability and the PR merge order before choosing the implementation base.
This specification does not close or merge either existing PR.

## Required implementation

- [ ] Add a repository CLI at `scripts/vision_logs.py`, using Python's standard
  library where practical, with `--help` and actionable errors.
- [ ] Make `bash scripts/run_front.sh ...` create a unique run directory and print
  its absolute path before starting ROS. Preserve argument boundaries and launch
  exit codes. Normalize shell scripts to LF through `.gitattributes`.
- [ ] Default storage to `~/.ros/robosub/logs/`, independent of working directory;
  allow `VISION_LOG_ROOT` to override it. Use timestamp plus a collision-resistant
  suffix for run IDs, and handle concurrent runs without overwriting evidence.
- [ ] Save console stdout and stderr, ROS launch/node logs, the startup summary,
  and Sydney's monitor output in the same run. Set `ROS_LOG_DIR` for child processes.
  Define how the monitor is started or explicitly attached to a run; setting an
  environment variable in one terminal does not capture another terminal's logs.
  Retain messages emitted before `/rosout` is available and Python tracebacks.
- [ ] Keep a metadata file: schema version, run ID, UTC start/end times, launch
  argument array, Git revision and dirty flag, ROS distribution, host/platform,
  live/SVO input, camera serial, mapping/localization mode, database path, effective
  topic names, configuration copies or hashes, and process exit statuses. Report
  unavailable values explicitly; do not dump the entire environment or credentials.
- [ ] Keep human-readable logs with timestamps, node/logger and severity when
  present. Provide one canonical search stream to avoid counting copies from
  console and ROS logs twice. Preserve unknown/unparsed lines rather than guessing
  a severity. Multiline tracebacks and summaries must remain readable.
- [ ] Emit actual warning severity when mapping extends an existing database,
  instead of only putting `WARN` inside a `LogInfo` message. Preserve custom serial
  and topic overrides in both the summary and real launch configuration.
- [ ] Flush output during execution and clean shutdown. Forward Ctrl+C/SIGTERM,
  allow RTAB-Map to save, reap child processes, and record aborted/failed runs.
  Document how incomplete runs after a crash or power loss are recognized.
- [ ] Bound disk usage: configurable per-file rotation and per-run cap, with a
  visible truncation/rotation notice. Offer age/size-based retention for completed
  runs. Do not silently remove old evidence or active logs to make room. Report
  disk-full or unwritable-root errors visibly and define whether launch proceeds.

## Proposed commands

Run these from the repository root after implementation. `latest` means the newest
run by recorded start time; `--all` searches all retained runs, including rotated
segments. Root overrides apply consistently to every command.

| Task | Proposed command |
| --- | --- |
| Launch and save logs | `bash scripts/run_front.sh start_rviz:=false` |
| List runs, status, age and disk usage | `python3 scripts/vision_logs.py list` |
| Show storage path and total usage | `python3 scripts/vision_logs.py usage` |
| Show latest run metadata and failure summary | `python3 scripts/vision_logs.py show latest` |
| Follow latest run live | `python3 scripts/vision_logs.py tail latest --follow` |
| All warnings across every run | `python3 scripts/vision_logs.py search --all --level WARN` |
| Errors and fatal messages in latest run | `python3 scripts/vision_logs.py search latest --min-level ERROR` |
| Warnings and errors together | `python3 scripts/vision_logs.py search latest --min-level WARN` |
| Search a symptom with surrounding lines | `python3 scripts/vision_logs.py search --all --text "data timed out" --context 3` |
| Narrow to one node/logger | `python3 scripts/vision_logs.py search latest --node vision_status` |
| Search TF problems | `python3 scripts/vision_logs.py search latest --text "extrapolation" --ignore-case --context 5` |
| Share one run for debugging | `python3 scripts/vision_logs.py export latest --output /tmp/vision-run.tar.gz` |
| Preview removal of runs older than 14 days | `python3 scripts/vision_logs.py clear --older-than 14d` |
| Delete those completed runs | `python3 scripts/vision_logs.py clear --older-than 14d --yes` |
| Preview clearing all completed logs | `python3 scripts/vision_logs.py clear --all` |
| Clear all completed logs | `python3 scripts/vision_logs.py clear --all --yes` |

Support an explicit run ID wherever `latest` appears. Search output should include
run ID, source file and line number, timestamp and severity where available. Define
exit codes (0 for matches/success, 1 for no matches, 2 for invalid input or I/O error).
Support a time range such as `--since` and `--until` with ISO-8601 timestamps.
Treat `--text` as literal text by default; make regex matching an explicit option.
Document whether node selection is exact and how unknown logger names are handled.

## Cleanup and export boundaries

- [ ] `clear` is a dry run unless `--yes` is supplied. Show full root path, selected
  run IDs, skipped active runs and bytes to reclaim before deletion.
- [ ] Delete only recognized managed run directories inside the resolved log root.
  Refuse symlink/path-traversal escapes and unsafe roots such as `/` or the home
  directory. Never delete maps, databases, SVOs, bags, source files, colcon logs or
  unrelated files. No broad `rm -rf ~/.ros` instructions.
- [ ] Protect active runs with a lock/liveness mechanism, including the race between
  selecting runs and deletion. A stale PID alone is insufficient because PIDs are
  reused. Explain safe handling of stale locks after a crash.
- [ ] Export logs and metadata only; never include camera recordings or map databases
  by following their paths in metadata. Reject overwrite unless explicitly requested.
  Avoid symlink escapes and indicate that paths/serials in a bundle may identify
  the test environment. Do not upload bundles automatically.

## Acceptance tests

Use temporary directories for cleanup tests; never clear real operator logs.

- [ ] Camera-free fake child emits INFO/WARN/ERROR, stderr, ANSI-colored output and
  a multiline traceback: saved evidence and search results are correct, with no
  duplicate counts. No-data and timeout events from Sydney's monitor are searchable.
- [ ] Two concurrent runs produce distinct folders; `latest`, explicit IDs and
  root overrides behave consistently, including paths containing spaces.
- [ ] Forced child failure preserves its nonzero exit status and logs. Ctrl+C
  and SIGTERM are forwarded and child processes exit cleanly.
- [ ] Rotation and caps work during a long synthetic run; searches include retained
  rotated segments. Permission-denied and disk-full behavior is tested or explicitly
  recorded as not tested, with a documented response.
- [ ] Cleanup preview leaves files unchanged. Confirmed cleanup deletes only eligible
  completed run directories, and preserves active runs, unrelated files, maps and
  recordings. Cover traversal, symlinks, unsafe roots, stale locks and races.
- [ ] Export contains the selected run's logs/metadata and no external linked files;
  the exported evidence can be inspected without ROS installed.
- [ ] Test startup summary fields, custom serial/topic forwarding, real warning
  severity, new versus existing database behavior, and mapping versus localization.
- [ ] Run `colcon test --packages-select vision_bringup vision_status` and
  `colcon test-result --verbose` when both packages are available. Add the new CLI
  tests to the documented test workflow and CI if available.
- [ ] Live-camera or SVO smoke test: start a run, cause and recover from a stream
  interruption, stop cleanly, then locate the corresponding warnings/recovery in
  saved logs. Record hardware-dependent checks as passed, failed or not run.
- [ ] Update README with implemented commands, storage/retention defaults, and a
  short troubleshooting walkthrough. Include sample output and actual test results
  in the PR before marking it ready for review.

## Useful follow-ups after the first version

Add warning/error counts grouped by node and message, first/last occurrence times,
and a comparison between a failed run and a known-good run. An optional diagnostic
snapshot can collect `ros2 node list`, topic types/publishers/QoS, selected parameters,
GPU availability and disk space, with timeouts so an unhealthy ROS graph cannot
hang collection. Keep recording images/bags, remote upload and a web dashboard out
of the initial scope.
