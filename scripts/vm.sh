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
#   rambla_sim_stop                      - kill sim-only processes
#   rambla_sim_status                    - show running sim-only processes
#   rambla_sim_logs [-f]                 - tail the sim log
#   rambla_stop_all                      - kill gateway + sim-only processes in one pass
#   rambla_vm_ps                         - show rambla/ros/gz-related processes on the VM
#   rambla_vm_health                     - uptime / load average / free memory
#   rambla_vm_sample <duration> [interval] - sample CPU/RSS on the VM, pull the CSV back

# Guard against double-sourcing (e.g. .zshrc + a manual `source`).
[[ -n "${_RAMBLA_VM_SH_LOADED:-}" ]] && return 0
_RAMBLA_VM_SH_LOADED=1

# --- Constants (intentionally not exported; only used within this file) ---

_RAMBLA_VM_HOST="rambla-vm"
_RAMBLA_GW_LOG="/tmp/rambla_gateway.log"
_RAMBLA_SIM_LOG="/tmp/rambla_sim.log"
_RAMBLA_GW_STATS="/tmp/rambla_gateway_stats.json"
_RAMBLA_SAMPLE_REMOTE="/tmp/rambla_sample.csv"
# Sim-only processes (Gazebo + ros_gz bridge/spawn + covariance injector).
# Deliberately excludes ekf_node/robot_state_publisher/safety_node, which are
# also used by a real (non-sim) gateway run -- see rambla_sim_stop below.
#
# Each alternative has one letter wrapped in a bracket (e.g. si[m] instead of
# sim). This is the standard pgrep self-match trick: `ssh host "pgrep -f
# 'sim'"` runs as a remote shell whose own command line contains the literal
# string "sim", so pgrep would otherwise match its own invocation. A bracket
# is a regex no-op against the real target process but never appears
# literally in the invoking command line, so it can't self-match.
_RAMBLA_SIM_PATTERN='gz si[m]|parameter_bridg[e]|ros_gz_si[m]|covariance_injecto[r]'
_RAMBLA_GW_PATTERN='rambla_control_pane[l]'

# Repo root, derived from this file's own location, so functions like
# rambla_vm_sample work no matter what directory you're in when you call them.
_RAMBLA_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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
  rambla_sim_stop                        kill sim-only processes (leaves shared real nodes alive)
  rambla_sim_status                      show running sim-only processes
  rambla_sim_logs [-f]                   tail the sim log (-f to follow)
  rambla_stop_all                        kill gateway + sim-only processes in one pass
  rambla_vm_ps                           show rambla/ros/gz-related processes on the VM
  rambla_vm_health                       uptime / load average / free memory
  rambla_vm_sample <duration> [interval] sample CPU/RSS on the VM, pull the CSV back
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
# Optional arg: world:=house (defaults to the apartment world).
rambla_sim_start() {
  # See the NOTE in rambla_gw_start: pass the whole remote command as a
  # single argument to ssh, not as separate "bash" "-lc" "<script>" args.
  ssh "$_RAMBLA_VM_HOST" "
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    nohup ros2 launch rambla_sim apartment_world.launch.py gui:=false $1 \
      > $_RAMBLA_SIM_LOG 2>&1 &
    disown
    echo \"sim started, pid \$!, log: $_RAMBLA_SIM_LOG\"
  "
}

# Kill sim-only processes (gz sim, ros_gz bridge/spawn, covariance_injector).
# Same pgrep-then-kill-9 pattern as rambla_gw_stop, for the same reasons.
#
# NOTE: ekf_node, robot_state_publisher, and safety_node are shared between
# the sim stack and a real (non-sim) gateway run, so they are deliberately
# NOT killed here -- doing so could break a gateway that's running
# concurrently. If you want to tear those down too, use rambla_stop_all or
# kill them manually.
rambla_sim_stop() {
  ssh "$_RAMBLA_VM_HOST" "
    pids=\$(pgrep -f '$_RAMBLA_SIM_PATTERN')
    if [ -z \"\$pids\" ]; then
      echo 'sim: no sim-only processes found'
    else
      echo \"killing sim-only pids: \$pids\"
      echo \"\$pids\" | xargs kill -9
      echo 'done (ekf_node/safety_node/robot_state_publisher left alive if running)'
    fi
  "
}

# Show running sim-only processes. Read-only, safe to call repeatedly.
rambla_sim_status() {
  ssh "$_RAMBLA_VM_HOST" "pgrep -fa '$_RAMBLA_SIM_PATTERN' || echo 'sim: no sim-only processes found'"
}

# Tail the sim log on the VM. Defaults to the last 50 lines; pass -f to
# follow (holds the SSH connection open until you Ctrl-C).
rambla_sim_logs() {
  ssh "$_RAMBLA_VM_HOST" "tail ${1:--n 50} $_RAMBLA_SIM_LOG"
}

# --- Combined / general -----------------------------------------------------

# Kill both the gateway and sim-only processes in one pass. Handy for
# clearing out orphaned processes from an earlier session before starting
# fresh. Same caveat as rambla_sim_stop: shared real nodes (ekf_node,
# robot_state_publisher, safety_node) are left alone.
rambla_stop_all() {
  ssh "$_RAMBLA_VM_HOST" "
    pids=\$(pgrep -f '$_RAMBLA_GW_PATTERN|$_RAMBLA_SIM_PATTERN')
    if [ -z \"\$pids\" ]; then
      echo 'nothing to kill'
    else
      echo \"killing pids: \$pids\"
      echo \"\$pids\" | xargs kill -9
      echo done
    fi
  "
}

# Show rambla/ros/gz-related processes on the VM (a filtered `ps aux`).
rambla_vm_ps() {
  ssh "$_RAMBLA_VM_HOST" "ps aux | grep -E 'rambla|ros2|gz|ekf|safety_node|parameter_bridge' | grep -v grep"
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
