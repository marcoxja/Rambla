# shellcheck shell=bash
#
# Convenience functions for interacting with the project's dev VM
# ("rambla-vm") over SSH, from the Mac.
#
# This file is meant to be SOURCED, not executed:
#
#   source scripts/vm.sh
#
# ...or add that line to your ~/.zshrc so the functions are always available.
# It assumes the SSH alias "rambla-vm" is already set up in ~/.ssh/config
# (key-based auth, user rambla_vm) -- this script does not manage that.
#
# Run `rambla_vm_help` after sourcing for a quick function reference.
#
# Function index:
#   rambla_vm_help                       - print this reference
#   rambla_vm_ssh [cmd]                  - interactive shell, or run a one-shot remote command
#   rambla_gw_start                      - launch the gateway (rambla_control_panel), detached
#   rambla_gw_stop                       - kill the gateway
#   rambla_gw_status                     - show running gateway processes
#   rambla_gw_logs [-f]                  - tail the gateway log
#   rambla_gw_stats                      - pretty-print gateway bandwidth stats
#   rambla_sim_start [world:=house]      - launch the Gazebo sim (headless), detached
#   rambla_sim_stop                      - tear down the whole sim/nav/localization/safety launch tree
#   rambla_sim_status                    - show the tracked launch tree + warn on any untracked orphans
#   rambla_sim_logs [-f]                 - tail the sim log
#   rambla_stop_all                      - kill gateway + the full sim launch tree in one pass
#   rambla_vm_ps                         - show rambla/ros2/gz-related processes on the VM (full tree, not just sim-only)
#   rambla_vm_health                     - uptime / load average / free memory
#   rambla_vm_sample <duration> [interval] - sample CPU/RSS on the VM, pull the CSV back
#   rambla_map_fetch <batch> [ver] [name]  - modal volume get a map_vNNN, stage + validate on the VM
#   rambla_map_validate <name>           - re-check a staged map's map.yaml/image on the VM
#   rambla_map_activate <name>           - validate, then atomically symlink it as the active map
#   rambla_map_list                      - show cached maps on the VM and which one is active
#   rambla_verify <scenario> [world]     - run a verify_localization scenario, pull the JSON report back
#   rambla_bag_record <name> [dur] [mb]  - record an observation batch on the VM with a live elapsed/size readout + auto-stop cap
#   rambla_bag_stop                      - manually stop an in-progress recording early
#   rambla_likelihood_capture [count] [max_wait_s] - capture (scan, amcl_pose, gz truth) snapshots for the M7 Phase 2.5 Task 4 diagnostic
#   rambla_likelihood_score <snapshots.json>       - score those snapshots against map_v002's likelihood field
#   rambla_convergence_sweep <label> [duration_s]  - M7 Phase 2.5 Task 5: formal head-to-head convergence-cycle sample for one amcl.yaml config

# Guard against double-sourcing (e.g. .zshrc + a manual `source`).
[[ -n "${_RAMBLA_VM_SH_LOADED:-}" ]] && return 0
_RAMBLA_VM_SH_LOADED=1

# --- Constants (intentionally not exported; only used within this file) ---

_RAMBLA_VM_HOST="rambla-vm"
_RAMBLA_GW_LOG="/tmp/rambla_gateway.log"
_RAMBLA_SIM_LOG="/tmp/rambla_sim.log"
_RAMBLA_SIM_PID_FILE="/tmp/rambla_sim.pid"
_RAMBLA_GW_STATS="/tmp/rambla_gateway_stats.json"
_RAMBLA_SAMPLE_REMOTE="/tmp/rambla_sample.csv"
# Recognizes every node apartment_world.launch.py's tree can spawn (sim +
# safety + localization + navigation + the launch process itself) --
# NOT the routine kill mechanism (that's the tracked PID/process-group in
# _RAMBLA_SIM_PID_FILE, which cascades cleanly to new nodes without this list
# needing to be kept in sync). This pattern exists only as (a) a fallback
# sweep in rambla_sim_stop for anything that detaches from the tracked group
# or predates PID tracking, and (b) rambla_sim_status/rambla_vm_ps orphan
# visibility -- see the 2026-07-16 M7 Phase 2 incident where a stale launch
# tree (duplicate safety_node/amcl/bt_navigator/ekf_node, each fighting the
# live one for the same topics/TF/action names) went completely unnoticed by
# both of those because neither covered nav2/localization node names at all.
#
# Each alternative has one letter wrapped in a bracket (e.g. si[m] instead of
# sim). This is the standard pgrep self-match trick: `ssh host "pgrep -f
# 'sim'"` runs as a remote shell whose own command line contains the literal
# string "sim", so pgrep would otherwise match its own invocation. A bracket
# is a regex no-op against the real target process but never appears
# literally in the invoking command line, so it can't self-match.
_RAMBLA_SIM_FULL_PATTERN='gz si[m]|parameter_bridg[e]|ros_gz_si[m]|covariance_injecto[r]|camera_compresso[r]|apartment_world\.launc[h]|safety_nod[e]|ekf_nod[e]|robot_state_publishe[r]|map_serve[r]|amc[l]|lifecycle_manage[r]|localization_monito[r]|localization_prob[e]|controller_serve[r]|planner_serve[r]|behavior_serve[r]|bt_navigato[r]|behavior_superviso[r]'
_RAMBLA_GW_PATTERN='rambla_control_pane[l]'
_RAMBLA_BAG_PID_FILE="/tmp/rambla_bag.pid"
_RAMBLA_BAG_LOG="/tmp/rambla_bag.log"
# Fallback-sweep pattern for rambla_bag_stop, same bracket self-match-avoidance
# trick as _RAMBLA_SIM_FULL_PATTERN above -- covers both the `ros2 launch`
# wrapper and the underlying `ros2 bag record` child in case they've become
# independent (e.g. the wrapper died but the recorder didn't, or vice versa).
_RAMBLA_BAG_PATTERN='ros2 ba[g]|record_observation_batch\.launc[h]'

# Repo root, derived from this file's own location, so functions like
# rambla_vm_sample work no matter what directory you're in when you call them.
# ${BASH_SOURCE[0]} is bash-only -- under zsh it's unset, which made dirname
# fall back to "." and silently resolve this relative to the caller's cwd
# instead of this file's actual location. ${(%):-%x} is zsh's equivalent.
if [ -n "$ZSH_VERSION" ]; then
  _rambla_vm_sh_path="${(%):-%x}"
else
  _rambla_vm_sh_path="${BASH_SOURCE[0]}"
fi
_RAMBLA_REPO_ROOT="$(cd "$(dirname "$_rambla_vm_sh_path")/.." && pwd)"
unset _rambla_vm_sh_path

# Print a quick reference of all functions in this file.
rambla_vm_help() {
  cat <<'EOF'
rambla-vm helper functions:

  rambla_vm_ssh [cmd]                    interactive shell, or run a one-shot remote command
  rambla_gw_start                        launch the gateway (rambla_control_panel), detached
  rambla_gw_stop                         kill the gateway
  rambla_gw_status                       show running gateway processes
  rambla_gw_logs [-f]                    tail the gateway log (-f to follow)
  rambla_gw_stats                        pretty-print gateway bandwidth stats
  rambla_sim_start [world:=house]        launch the Gazebo sim (headless), detached
  rambla_sim_stop                        tear down the whole sim/nav/localization/safety launch tree
  rambla_sim_status                      show the tracked launch tree + warn on any untracked orphans
  rambla_sim_logs [-f]                   tail the sim log (-f to follow)
  rambla_stop_all                        kill gateway + the full sim launch tree in one pass
  rambla_vm_ps                           show rambla/ros2/gz-related processes on the VM (full tree)
  rambla_vm_health                       uptime / load average / free memory
  rambla_vm_sample <duration> [interval] sample CPU/RSS on the VM, pull the CSV back
  rambla_map_fetch <batch> [ver] [name]  modal volume get a map_vNNN, stage + validate on the VM
  rambla_map_validate <name>             re-check a staged map's map.yaml/image on the VM
  rambla_map_activate <name>             validate, then atomically symlink it as the active map
  rambla_map_list                        show cached maps on the VM and which one is active
  rambla_verify <scenario> [world]       run a verify_localization scenario, pull the JSON report back
  rambla_bag_record <name> [dur] [mb]    record an observation batch on the VM, live elapsed/size readout + auto-stop cap
  rambla_bag_stop                        manually stop an in-progress recording early
  rambla_likelihood_capture [count] [max_wait_s]  capture snapshots for the M7 Phase 2.5 Task 4 likelihood diagnostic
  rambla_likelihood_score <snapshots.json>        score captured snapshots against map_v002's likelihood field
  rambla_convergence_sweep <label> [duration_s]   M7 Phase 2.5 Task 5: sample convergence-cycle stats for the currently-deployed amcl.yaml
EOF
}

# Open an interactive SSH session to the VM, or, if args are given, run them
# as a single remote command and return. This is the primitive every other
# function in this file builds on.
rambla_vm_ssh() {
  ssh "$_RAMBLA_VM_HOST" "$@"
}

# --- Gateway (rambla_control_panel) ---------------------------------------

# Launch the gateway on the VM, detached, so this returns immediately and the
# process keeps running after the SSH connection closes. Credentials
# (ROBOT_TOKEN, RAMBLA_RELAY_URL, RAMBLA_ROBOT_ID) live in ~/.rambla_relay.env
# on the VM and are sourced there -- never read or printed on the Mac side.
rambla_gw_start() {
  # NOTE: the remote command is passed as a single argument to ssh (not as
  # separate "bash" "-lc" "<script>" args) -- ssh joins multiple trailing
  # arguments with spaces and re-parses them as one string on the remote
  # side, so passing them separately would not preserve this as one script.
  ssh "$_RAMBLA_VM_HOST" "
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    source ~/.rambla_relay.env
    nohup ros2 launch rambla_control_panel control_panel.launch.py \
      relay_url:=\"\$RAMBLA_RELAY_URL\" robot_token:=\"\$ROBOT_TOKEN\" robot_id:=\"\$RAMBLA_ROBOT_ID\" \
      > $_RAMBLA_GW_LOG 2>&1 &
    disown
    echo \"gateway started, pid \$!, log: $_RAMBLA_GW_LOG\"
  "
}

# Kill the gateway. Uses pgrep to list PIDs and kill -9 on each one, rather
# than `pkill -f`, because pattern-based pkill has been unreliable on this
# VM. SIGKILL (not SIGTERM) is required because the gateway's rclpy process
# ignores SIGTERM when backgrounded via nohup.
rambla_gw_stop() {
  ssh "$_RAMBLA_VM_HOST" "
    pids=\$(pgrep -f '$_RAMBLA_GW_PATTERN')
    if [ -z \"\$pids\" ]; then
      echo 'gateway: no matching processes found'
    else
      echo \"killing pids: \$pids\"
      echo \"\$pids\" | xargs kill -9
      echo done
    fi
  "
}

# Show running gateway processes (full command lines). Read-only, safe to
# call repeatedly.
rambla_gw_status() {
  ssh "$_RAMBLA_VM_HOST" "pgrep -fa '$_RAMBLA_GW_PATTERN' || echo 'gateway: not running'"
}

# Tail the gateway log on the VM. Defaults to the last 50 lines; pass -f to
# follow (this holds the SSH connection open until you Ctrl-C).
rambla_gw_logs() {
  ssh "$_RAMBLA_VM_HOST" "tail ${1:--n 50} $_RAMBLA_GW_LOG"
}

# Pretty-print the gateway's bandwidth stats JSON for a quick sanity check
# (no need to run the full resource sampler for this).
rambla_gw_stats() {
  ssh "$_RAMBLA_VM_HOST" "cat $_RAMBLA_GW_STATS | python3 -m json.tool"
}

# --- Simulation (Gazebo) ----------------------------------------------------

# Launch the Gazebo sim on the VM, headless, detached. gui:=false is
# mandatory here since there's no DISPLAY over SSH -- for the GUI, use the
# VM console directly (startx), not this function.
# Optional arg: world:=house (default and only supported world -- the old
# apartment_world placeholder was removed; see docs/getting-started-sim.md).
rambla_sim_start() {
  # See the NOTE in rambla_gw_start: pass the whole remote command as a
  # single argument to ssh, not as separate "bash" "-lc" "<script>" args.
  #
  # setsid gives the launch its own session/process group (PGID == its own
  # PID), so rambla_sim_stop can reliably tear down the *entire* tree --
  # apartment_world.launch.py now bundles sim + safety + localization +
  # navigation into one `ros2 launch` invocation, and every node it spawns
  # inherits this group unless it explicitly detaches. The PID is persisted
  # to _RAMBLA_SIM_PID_FILE (not just echoed) so rambla_sim_stop/_status can
  # find it from a fresh shell/session, and this function refuses to start a
  # second stack on top of a still-live one -- exactly the double-launch
  # that went unnoticed and left two full node trees (duplicate
  # safety_node/amcl/bt_navigator, etc.) fighting each other, 2026-07-16.
  ssh "$_RAMBLA_VM_HOST" "
    if [ -f $_RAMBLA_SIM_PID_FILE ] && kill -0 \$(cat $_RAMBLA_SIM_PID_FILE) 2>/dev/null; then
      echo \"sim: a previous run (pid \$(cat $_RAMBLA_SIM_PID_FILE)) is still alive - run rambla_sim_stop first\" >&2
      exit 1
    fi
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    setsid nohup ros2 launch rambla_sim apartment_world.launch.py gui:=false $1 \
      > $_RAMBLA_SIM_LOG 2>&1 < /dev/null &
    pid=\$!
    disown
    echo \$pid > $_RAMBLA_SIM_PID_FILE
    echo \"sim started, pid \$pid, log: $_RAMBLA_SIM_LOG\"
  "
}

# Tear down the *entire* launch tree rambla_sim_start created (sim + safety +
# localization + navigation - apartment_world.launch.py bundles all of it
# into one `ros2 launch` invocation now, so there is no independently-running
# "real gateway" layer left to protect by sparing individual node names; see
# the constants block above and the 2026-07-16 incident note).
#
# Primary path: SIGINT the tracked PID (graceful `ros2 launch` shutdown -
# the same cascade Ctrl+C triggers interactively, which is more reliable
# than guessing this launch's internal process-group topology), wait, then
# escalate to SIGTERM/SIGKILL on both the PID and its process group if it
# hasn't exited. Fallback path: sweep _RAMBLA_SIM_FULL_PATTERN for any
# straggler regardless of ancestry - catches nodes that detached from the
# tracked group, or (a one-time concern) processes left over from before
# this PID-tracking mechanism existed.
rambla_sim_stop() {
  ssh "$_RAMBLA_VM_HOST" "
    if [ -f $_RAMBLA_SIM_PID_FILE ]; then
      pid=\$(cat $_RAMBLA_SIM_PID_FILE)
      if kill -0 \$pid 2>/dev/null; then
        echo \"stopping tracked launch tree (pid \$pid): sending SIGINT\"
        kill -INT \$pid 2>/dev/null
        for i in 1 2 3 4 5 6; do
          kill -0 \$pid 2>/dev/null || break
          sleep 1
        done
        if kill -0 \$pid 2>/dev/null; then
          echo \"pid \$pid still alive after SIGINT - escalating to SIGKILL (pid + process group)\"
          kill -KILL \$pid 2>/dev/null
          kill -KILL -\$pid 2>/dev/null
        fi
      fi
      rm -f $_RAMBLA_SIM_PID_FILE
    else
      echo 'sim: no tracked launch pid found'
    fi
    strays=\$(pgrep -f '$_RAMBLA_SIM_FULL_PATTERN')
    if [ -n \"\$strays\" ]; then
      echo \"killing stray/untracked pids: \$strays\"
      echo \"\$strays\" | xargs kill -9
    fi
    echo done
  "
}

# Show the tracked launch tree's state plus a read-only orphan sweep (any
# process matching _RAMBLA_SIM_FULL_PATTERN NOT started by the tracked pid -
# should always be empty; if not, rambla_sim_stop's fallback sweep will
# clean it up). Safe to call repeatedly.
rambla_sim_status() {
  ssh "$_RAMBLA_VM_HOST" "
    if [ -f $_RAMBLA_SIM_PID_FILE ] && kill -0 \$(cat $_RAMBLA_SIM_PID_FILE) 2>/dev/null; then
      echo \"tracked launch: pid \$(cat $_RAMBLA_SIM_PID_FILE) alive\"
    else
      echo 'tracked launch: none alive'
    fi
    echo '--- full tree (sim + safety + localization + navigation) ---'
    pgrep -fa '$_RAMBLA_SIM_FULL_PATTERN' || echo '(nothing found)'
  "
}

# Tail the sim log on the VM. Defaults to the last 50 lines; pass -f to
# follow (holds the SSH connection open until you Ctrl-C).
rambla_sim_logs() {
  ssh "$_RAMBLA_VM_HOST" "tail ${1:--n 50} $_RAMBLA_SIM_LOG"
}

# --- Combined / general -----------------------------------------------------

# Kill both the gateway and the full sim launch tree in one pass. Handy for
# clearing out orphaned processes from an earlier session before starting
# fresh.
rambla_stop_all() {
  rambla_sim_stop
  ssh "$_RAMBLA_VM_HOST" "
    pids=\$(pgrep -f '$_RAMBLA_GW_PATTERN')
    if [ -z \"\$pids\" ]; then
      echo 'gateway: nothing to kill'
    else
      echo \"killing gateway pids: \$pids\"
      echo \"\$pids\" | xargs kill -9
    fi
  "
}

# Show rambla/ros2/gz-related processes on the VM (a filtered `ps aux`).
# Uses the full sim-tree pattern (not just gz/ekf/safety_node) so nav2 and
# localization nodes (amcl, bt_navigator, controller_server, ...) actually
# show up here too -- see the 2026-07-16 incident note above.
rambla_vm_ps() {
  ssh "$_RAMBLA_VM_HOST" "ps aux | grep -E 'rambla|$_RAMBLA_SIM_FULL_PATTERN' | grep -v grep"
}

# Quick VM health snapshot: uptime, load average, free memory.
rambla_vm_health() {
  ssh "$_RAMBLA_VM_HOST" "uptime && echo --- && cat /proc/loadavg && echo --- && free -h"
}

# Run scripts/sample_resources.py on the VM for <duration> seconds
# (optional <interval>, default 1s) and pull the resulting CSV back to the
# current directory with a timestamped filename. The script is piped over
# SSH stdin rather than copied to the VM first, per its own usage docstring
# -- no separate deploy step needed.
rambla_vm_sample() {
  local duration="$1"
  local interval="${2:-1}"
  if [ -z "$duration" ]; then
    echo "usage: rambla_vm_sample <duration_secs> [interval_secs]" >&2
    return 1
  fi

  local sample_script="$_RAMBLA_REPO_ROOT/scripts/sample_resources.py"
  local local_out
  local_out="./rambla_sample_$(date +%Y%m%d_%H%M%S).csv"

  ssh "$_RAMBLA_VM_HOST" python3 - --duration "$duration" --interval "$interval" --out "$_RAMBLA_SAMPLE_REMOTE" < "$sample_script" \
    && scp "$_RAMBLA_VM_HOST:$_RAMBLA_SAMPLE_REMOTE" "$local_out" \
    && echo "saved: $local_out"
}

# --- Map artifact cache (M5 Phase 2) ----------------------------------------
#
# The Pi/VM-local localization stack (M5 Phase 3+) reads a fixed path,
# ~/ros2_ws/maps/active/map.yaml, and knows nothing about Modal volumes or
# batch names -- mirrors INT-001's "application layers don't care what's
# underneath" principle. These functions are the deliberate, manual bridge:
# fetch a map_vNNN produced by src/compute/slam/modal_app.py's
# run_mapping_job off the `rambla-slam-data` Modal Volume, stage + validate
# it on rambla-vm, then explicitly activate it via an atomic symlink swap.
# No automatic discovery/polling/cloud sync -- out of scope by design.
#
# `modal volume get`/`put` run from the Mac (this project's existing
# convention -- see .claude/internal-docs/compute/slam/CLAUDE.md's
# data-transfer notes), so rambla_map_fetch downloads to a local scratch dir
# first, then scp's on to rambla-vm.

_RAMBLA_MAP_FETCH_TMP="/tmp/rambla_map_fetch"
_RAMBLA_VM_MAPS_DIR="~/ros2_ws/maps"

# Download a map_vNNN artifact and stage it on rambla-vm at
# ~/ros2_ws/maps/<local_name>/, then validate it there (same checks
# modal_app.py's _verify_map_yaml already ran on Modal -- re-run here in
# case the modal-volume-get + scp transfer itself corrupted something).
# Does NOT activate it -- see rambla_map_activate, a separate deliberate
# step.
#
# Usage: rambla_map_fetch <batch_name> [map_version] [local_name]
#   batch_name   the Modal Volume's /outputs/<batch_name>/ directory
#   map_version  which map_vNNN under that batch (default: map_v001)
#   local_name   directory name to use under ~/ros2_ws/maps/ on rambla-vm
#                (default: same as map_version) -- never overwritten if it
#                already exists there; pass a distinct local_name to stage
#                a different batch's map alongside an existing one.
rambla_map_fetch() {
  local batch_name="$1"
  local map_version="${2:-map_v001}"
  local local_name="${3:-$map_version}"
  if [ -z "$batch_name" ]; then
    echo "usage: rambla_map_fetch <batch_name> [map_version] [local_name]" >&2
    return 1
  fi

  if ssh "$_RAMBLA_VM_HOST" "[ -e $_RAMBLA_VM_MAPS_DIR/$local_name ]"; then
    echo "refuse: rambla-vm:$_RAMBLA_VM_MAPS_DIR/$local_name already exists -- pass a distinct local_name to fetch alongside it" >&2
    return 1
  fi

  local local_tmp="$_RAMBLA_MAP_FETCH_TMP/$local_name"
  local fetch_staging="$_RAMBLA_MAP_FETCH_TMP/.staging"
  rm -rf "$local_tmp" "$fetch_staging"
  mkdir -p "$fetch_staging"
  echo "modal volume get: /outputs/$batch_name/$map_version -> $local_tmp"
  # modal volume get (client 1.5.2) only populates a directory download
  # correctly when the local destination already exists as a directory --
  # nesting the result one level under its basename inside it. Given a
  # non-existent destination it silently flattens every file in the remote
  # directory onto that single path instead (last file downloaded wins),
  # so we always download into a pre-created staging dir and move the
  # nested result into place, regardless of local_name vs map_version.
  modal volume get rambla-slam-data "/outputs/$batch_name/$map_version" "$fetch_staging" || return 1
  mv "$fetch_staging/$(basename "$map_version")" "$local_tmp" || return 1
  rm -rf "$fetch_staging"

  echo "scp: $local_tmp -> rambla-vm:$_RAMBLA_VM_MAPS_DIR/$local_name"
  ssh "$_RAMBLA_VM_HOST" "mkdir -p $_RAMBLA_VM_MAPS_DIR" || return 1
  scp -rq "$local_tmp" "$_RAMBLA_VM_HOST:$_RAMBLA_VM_MAPS_DIR/$local_name" || return 1
  rm -rf "$local_tmp"

  rambla_map_validate "$local_name"
}

# Re-run the map.yaml/resolution/origin/image checks (scripts/validate_map.py,
# mirroring modal_app.py's _verify_map_yaml) against a map already staged on
# rambla-vm. Piped over SSH stdin, same pattern as rambla_vm_sample below --
# no scp/deploy step needed.
#
# Usage: rambla_map_validate <local_name>
rambla_map_validate() {
  local local_name="$1"
  if [ -z "$local_name" ]; then
    echo "usage: rambla_map_validate <local_name>" >&2
    return 1
  fi
  local validate_script="$_RAMBLA_REPO_ROOT/scripts/validate_map.py"
  ssh "$_RAMBLA_VM_HOST" python3 - --map-dir "$_RAMBLA_VM_MAPS_DIR/$local_name" < "$validate_script"
}

# Atomically activate a staged map as the one localize.launch.py's
# map_server points at (~/ros2_ws/maps/active, a fixed path). Deliberately a
# separate, manual step from rambla_map_fetch -- see the section header
# above. Re-validates immediately before the swap (not just trusting an
# earlier fetch) so a broken/corrupted map is never made active.
#
# Usage: rambla_map_activate <local_name>
rambla_map_activate() {
  local local_name="$1"
  if [ -z "$local_name" ]; then
    echo "usage: rambla_map_activate <local_name>" >&2
    return 1
  fi
  rambla_map_validate "$local_name" || {
    echo "refuse: $local_name failed validation -- not activating" >&2
    return 1
  }
  ssh "$_RAMBLA_VM_HOST" "ln -sfn $_RAMBLA_VM_MAPS_DIR/$local_name $_RAMBLA_VM_MAPS_DIR/active" || return 1
  echo "activated: $local_name -> $_RAMBLA_VM_MAPS_DIR/active"
}

# Show the map_vNNN directories cached on rambla-vm and which one (if any)
# is currently active. Read-only, safe to call repeatedly.
rambla_map_list() {
  ssh "$_RAMBLA_VM_HOST" "
    echo 'cached maps:'
    ls -1 $_RAMBLA_VM_MAPS_DIR 2>/dev/null | grep -v '^active\$' || echo '  (none)'
    echo
    if [ -L $_RAMBLA_VM_MAPS_DIR/active ]; then
      echo \"active -> \$(readlink $_RAMBLA_VM_MAPS_DIR/active)\"
    else
      echo 'active: (not set)'
    fi
  "
}

# --- Verification harness (M5 Phase 7) --------------------------------------
#
# rambla_localization/verify_localization.py runs a deterministic scenario
# against a sim + localize.launch.py stack already started via
# rambla_sim_start, then writes a small event log + verification_report.json
# instead of raw per-tick telemetry -- see M5_PLAN.md's Verification section
# for why (manual command-by-command monitoring proved too slow/noisy).
#
# Requires the VM's checkout to already be rebuilt after any
# rambla_localization change:
#   rambla_vm_ssh "source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build --packages-select rambla_localization"

_RAMBLA_VERIFY_REMOTE="/tmp/rambla_verify_report.json"

# Run one verify_localization scenario on the VM and pull the JSON report
# back to the Mac. Blocks in the foreground for the scenario's duration (same
# as rambla_vm_sample) -- these are real timed sim runs, not instant checks.
#
# Usage: rambla_verify <scenario> [world] [extra verify_localization args...]
#   scenario  cold_start | kidnap | ekf_drift
#   world     defaults to house -- must match whatever rambla_sim_start used
#   extras    passed through verbatim, e.g. --converge-timeout-s 300
rambla_verify() {
  local scenario="$1"
  if [ -z "$scenario" ]; then
    echo "usage: rambla_verify <scenario> [world] [extra args...]" >&2
    return 1
  fi
  shift

  local world="house"
  if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then
    # $1 doesn't start with "--" -- treat it as the world positional arg.
    world="$1"
    shift
  fi
  # "$@" is now just the passthrough extras, e.g. --converge-timeout-s 300

  ssh "$_RAMBLA_VM_HOST" "
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run rambla_localization verify_localization \
      --scenario $scenario --world $world --report-path $_RAMBLA_VERIFY_REMOTE $*
  "
  # NOTE: `status` is a read-only special parameter in zsh (an alias for
  # $?) -- naming a local that would fail with "read-only variable: status".
  local exit_code=$?

  local local_out="./rambla_verify_${scenario}_$(date +%Y%m%d_%H%M%S).json"
  scp -q "$_RAMBLA_VM_HOST:$_RAMBLA_VERIFY_REMOTE" "$local_out" \
    && echo "report saved: $local_out"

  return $exit_code
}

# --- Observation-batch recording (M7 map coverage checkpoint) ---------------
#
# rosbag2 cannot self-stop (--max-bag-size/--max-bag-duration only *split*
# files, there's no stop-after flag, and `ros2 bag info` can't read a live
# recording) -- rambla_bag_record is the external wrapper that polls size/
# elapsed and stops it, mirroring rambla_sim_start/_stop's tracked-PID idiom.
# The recorder itself must be stopped with SIGTERM, not SIGINT: a `nohup
# <cmd> &`-backgrounded process on rambla-vm inherits SIG_IGN for SIGINT, and
# only SIGTERM reaches rclcpp's shutdown handler to flush/close the bag (see
# .claude/internal-docs/compute/slam/CLAUDE.md's M3 finding).

# Start an observation-batch recording on rambla-vm and block in the
# foreground (pty-allocated, so Ctrl-C here reaches the remote wrapper)
# printing an elapsed/size status line every ~5s. Auto-stops the recorder
# (SIGTERM, escalating to SIGKILL) when either cap is hit, on Ctrl-C, or when
# rambla_bag_stop is run from another terminal -- in that last case this
# function's own next poll just notices the recorder is gone and returns.
# Refuses to start a second recording while one is already tracked-alive,
# same guard as rambla_sim_start.
#
# Usage: rambla_bag_record <name> [max_duration_s=360] [max_size_mb=250]
#   name  subdirectory under ~/rambla_bags/ on rambla-vm for this batch
rambla_bag_record() {
  local name="$1"
  local max_duration_s="${2:-360}"
  local max_size_mb="${3:-250}"
  if [ -z "$name" ]; then
    echo "usage: rambla_bag_record <name> [max_duration_s=360] [max_size_mb=250]" >&2
    return 1
  fi

  # $HOME (not a leading ~) because this gets glued onto
  # "observation_batch_dir:=" as one shell word on the remote side -- tilde
  # expansion only triggers when ~ is the first character of a word, but
  # $HOME expands anywhere.
  local batch_dir="\$HOME/rambla_bags/$name"

  ssh -t "$_RAMBLA_VM_HOST" "
    if [ -f $_RAMBLA_BAG_PID_FILE ] && kill -0 \$(cat $_RAMBLA_BAG_PID_FILE) 2>/dev/null; then
      echo \"bag: a previous recording (pid \$(cat $_RAMBLA_BAG_PID_FILE)) is still tracked-alive - run rambla_bag_stop first\" >&2
      exit 1
    fi

    stop_recorder() {
      if [ -f $_RAMBLA_BAG_PID_FILE ]; then
        pid=\$(cat $_RAMBLA_BAG_PID_FILE)
        if kill -0 \$pid 2>/dev/null; then
          echo \"stopping recorder (pid \$pid): sending SIGTERM\"
          kill -TERM \$pid 2>/dev/null
          for i in 1 2 3 4 5 6; do
            kill -0 \$pid 2>/dev/null || break
            sleep 1
          done
          if kill -0 \$pid 2>/dev/null; then
            echo \"pid \$pid still alive after SIGTERM - escalating to SIGKILL (pid + process group)\"
            kill -KILL \$pid 2>/dev/null
            kill -KILL -\$pid 2>/dev/null
          fi
        fi
        rm -f $_RAMBLA_BAG_PID_FILE
      fi
    }
    trap 'echo; echo \"caught INT/TERM - stopping recorder\"; stop_recorder; exit 130' INT TERM

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    setsid nohup ros2 launch rambla_bagging record_observation_batch.launch.py \
      observation_batch_dir:=$batch_dir \
      > $_RAMBLA_BAG_LOG 2>&1 < /dev/null &
    pid=\$!
    disown
    echo \$pid > $_RAMBLA_BAG_PID_FILE
    echo \"recording started: pid \$pid, dir $batch_dir, log $_RAMBLA_BAG_LOG\"
    echo \"cap: ${max_duration_s}s / ${max_size_mb}MB - Ctrl-C to stop early\"

    start=\$(date +%s)
    while true; do
      sleep 5
      if ! kill -0 \$pid 2>/dev/null; then
        echo \"recorder (pid \$pid) exited on its own - check $_RAMBLA_BAG_LOG\"
        rm -f $_RAMBLA_BAG_PID_FILE
        break
      fi
      elapsed=\$(( \$(date +%s) - start ))
      size_mb=\$(du -sm $batch_dir 2>/dev/null | cut -f1)
      size_mb=\${size_mb:-0}
      echo \"elapsed \${elapsed}s/${max_duration_s}s, size \${size_mb}MB/${max_size_mb}MB (remaining \$(( $max_duration_s - elapsed ))s / \$(( $max_size_mb - size_mb ))MB)\"
      if [ \$elapsed -ge $max_duration_s ] || [ \$size_mb -ge $max_size_mb ]; then
        echo \"cap reached - auto-stopping\"
        stop_recorder
        break
      fi
    done
  "
}

# Manually stop an in-progress rambla_bag_record recording early (e.g. once
# you've driven the full mapping route and don't want to wait for a timer/
# size cap). Kills the tracked PID explicitly (SIGTERM, escalating to
# SIGKILL) rather than a pattern pkill -- pattern-based pkill has been
# unreliable on this VM -- then falls back to a pgrep-listed sweep of
# _RAMBLA_BAG_PATTERN for any untracked straggler, same two-tier shape as
# rambla_sim_stop.
rambla_bag_stop() {
  ssh "$_RAMBLA_VM_HOST" "
    if [ -f $_RAMBLA_BAG_PID_FILE ]; then
      pid=\$(cat $_RAMBLA_BAG_PID_FILE)
      if kill -0 \$pid 2>/dev/null; then
        echo \"stopping recorder (pid \$pid): sending SIGTERM\"
        kill -TERM \$pid 2>/dev/null
        for i in 1 2 3 4 5 6; do
          kill -0 \$pid 2>/dev/null || break
          sleep 1
        done
        if kill -0 \$pid 2>/dev/null; then
          echo \"pid \$pid still alive after SIGTERM - escalating to SIGKILL (pid + process group)\"
          kill -KILL \$pid 2>/dev/null
          kill -KILL -\$pid 2>/dev/null
        fi
      else
        echo 'bag: tracked pid not alive'
      fi
      rm -f $_RAMBLA_BAG_PID_FILE
    else
      echo 'bag: no tracked recording pid found'
    fi
    strays=\$(pgrep -f '$_RAMBLA_BAG_PATTERN')
    if [ -n \"\$strays\" ]; then
      echo \"killing stray/untracked pids: \$strays\"
      echo \"\$strays\" | xargs kill -9
    fi
    echo done
  "
}

# --- Localization convergence diagnostic (M7 Phase 2.5 Task 4) --------------
#
# Offline likelihood-scoring diagnostic: does AMCL's own measurement model
# score the ground-truth pose better than the wrong pose it actually
# converged to (a tuning/sampling problem), or comparably/worse (genuine
# perceptual aliasing that no AMCL parameter tuning can fix)? See
# M7_LOCALIZATION_FINDINGS.md section 6 and M7_PLAN.md's Phase 2.5 Task 4.
# Two steps, both piped over SSH stdin like rambla_map_validate/
# rambla_vm_sample -- no colcon build/deploy needed, this is throwaway
# diagnostic tooling, not a shipped verification scenario.

_RAMBLA_LIKELIHOOD_SNAPSHOTS_REMOTE="/tmp/rambla_likelihood_snapshots.json"

# Step 1: watch the already-running sim (localize:=true) and capture up to
# <count> (scan, amcl_pose, gz-ground-truth) snapshots, one per natural
# EXHAUSTED-while-pre-USABLE edge -- no restart, no teleport, just passive
# observation across whatever GLOBAL/CONVERGING cycles the sim is already
# doing. Blocks in the foreground for up to <max_wait_s> (default 600s).
rambla_likelihood_capture() {
  local count="${1:-4}"
  local max_wait_s="${2:-600}"
  local capture_script="$_RAMBLA_REPO_ROOT/scripts/likelihood_capture.py"
  local local_out="./rambla_likelihood_snapshots_$(date +%Y%m%d_%H%M%S).json"

  ssh "$_RAMBLA_VM_HOST" "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && python3 - --count $count --max-wait-s $max_wait_s" \
    < "$capture_script" > "$local_out" \
    && scp -q "$local_out" "$_RAMBLA_VM_HOST:$_RAMBLA_LIKELIHOOD_SNAPSHOTS_REMOTE" \
    && echo "snapshots saved: $local_out (also staged on VM: $_RAMBLA_LIKELIHOOD_SNAPSHOTS_REMOTE)"
}

# Step 2: score the captured snapshots against map_v002's likelihood field
# (map.pgm + the installed amcl.yaml, both already on the VM -- runs there
# since scipy/numpy are already installed there, not on the Mac).
#
# Usage: rambla_likelihood_score [local_snapshots.json]
#   defaults to whatever rambla_likelihood_capture last staged on the VM.
rambla_likelihood_score() {
  local local_snapshots="$1"
  local score_script="$_RAMBLA_REPO_ROOT/scripts/likelihood_score.py"
  local remote_snapshots="$_RAMBLA_LIKELIHOOD_SNAPSHOTS_REMOTE"

  if [ -n "$local_snapshots" ]; then
    remote_snapshots="/tmp/$(basename "$local_snapshots")"
    scp -q "$local_snapshots" "$_RAMBLA_VM_HOST:$remote_snapshots" || return 1
  fi

  # No ROS sourcing needed here (unlike capture) -- this step is pure
  # numpy/scipy + flat-file parsing, no rclpy import.
  ssh "$_RAMBLA_VM_HOST" "python3 - --snapshots $remote_snapshots --report-path /tmp/rambla_likelihood_report.json" \
    < "$score_script"
  local exit_code=$?

  local local_report="./rambla_likelihood_report_$(date +%Y%m%d_%H%M%S).json"
  scp -q "$_RAMBLA_VM_HOST:/tmp/rambla_likelihood_report.json" "$local_report" \
    && echo "report saved: $local_report"

  return $exit_code
}

# Passive convergence-cycle sample against whatever amcl.yaml is currently
# built+deployed on the VM (rebuild it yourself first if you just edited
# it -- this does not colcon build or restart the sim). Blocks in the
# foreground for <duration_s> (default 900s); <label> is just recorded in
# the report so head-to-head runs are distinguishable afterward.
#
# Usage: rambla_convergence_sweep <label> [duration_s]
rambla_convergence_sweep() {
  local label="$1"
  local duration_s="${2:-900}"
  if [ -z "$label" ]; then
    echo "usage: rambla_convergence_sweep <label> [duration_s]" >&2
    return 1
  fi
  local sweep_script="$_RAMBLA_REPO_ROOT/scripts/convergence_sweep.py"
  local local_out="./rambla_sweep_${label}_$(date +%Y%m%d_%H%M%S).json"

  ssh "$_RAMBLA_VM_HOST" "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && python3 - --duration-s $duration_s --label $label" \
    < "$sweep_script" > "$local_out" \
    && echo "sweep report saved: $local_out"
}
