#!/usr/bin/env bash
# Usage: run.sh <label> <vanilla 0|1> [seconds] [no_chunk_group 0|1] [dps] [activation always|auto] [mode kinematic|frame]
# Launches OUR OWN SE2 instance (-allowMultiple, -plugins:D:\SpinProbe...), waits for the probe to finish (or the crash
# handler / our process exit), copies the probe + game logs to runs/<label>.*.
# Only processes whose command line contains "SpinProbe" are ever inspected or killed: other agents run their own SE2.
label=$1; vanilla=$2; seconds=${3:-90}
SRC=/d/SpinProbe/obj/Release/net9.0; OUT=/d/SpinProbe/bin/Release/net9.0; B=/d/SpinProbe/live/$label; G=/c/SpinProbe-run/appdata/Temp/Logs; mkdir -p /d/SpinProbe/runs
EXE='D:\SteamLibrary\steamapps\common\SpaceEngineers2\Game2\SpaceEngineers2.exe'

ours() { # PIDs of SE2 processes started with our plugin
  powershell.exe -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='SpaceEngineers2.exe'\" | Where-Object { \$_.CommandLine -like '*SpinProbe*' } | ForEach-Object { \$_.ProcessId }" 2>/dev/null | tr -d '\r'
}
kill_ours() { for p in $(ours); do taskkill //F //PID $p >/dev/null 2>&1; done; }

kill_ours; for w in $(seq 1 30); do [ -z "$(ours)" ] && break; sleep 2; done; sleep 3
# Each run loads its own copy of the plugin, so the next build never fights a running game for the DLL.
rm -rf $B; mkdir -p $B; cp $OUT/*.dll $OUT/*.json $B/ 2>/dev/null; cp $SRC/ChildlessParents.dll $B/
printf "dps=%s
seconds=%s
vanilla=%s
no_chunk_group=%s
activation=%s
mode=%s
audit=%s
save=%s
spin_dps=%s
" "${5:-0.02}" "$seconds" "$vanilla" "${4:-0}" "${6:-always}" "${7:-kinematic}" "${8:-0}" "${9:-Welcome To Verdure 1 47}" "${10:-0.5}" > $B/spinprobe.ini
# Extra settings: EXTRA_INI holds ini lines separated by a literal backslash-n (see the run notes).
[ -n "$EXTRA_INI" ] && printf '%b\n' "$EXTRA_INI" >> $B/spinprobe.ini
# Built from quoted pieces: inside double quotes a backslash before $label escapes it (runs 18/19 loaded no plugin).
PLUG='D:\SpinProbe\live\'"$label"'\ChildlessParents.dll'
# The game log of THIS run: the newest SE2 log written since our launch whose command line names our plugin folder.
# Before, the newest log in the shared folder was taken blindly, which can be another instance's.
ourlog() {
  local l
  for l in $(ls -t $G/SpaceEngineers2_*.log 2>/dev/null | grep -v -E "_(Mission|Render12|Stats)"); do
    [ "$(stat -c %Y "$l")" -ge "$start" ] || break
    head -c 4000 "$l" | grep -qE "live.${label}.ChildlessParents" && { echo "$l"; return; }
  done
}
launch() {
  start=$(date +%s)
  powershell.exe -NoProfile -Command "Start-Process -FilePath '$EXE' -WorkingDirectory 'D:\SteamLibrary\steamapps\common\SpaceEngineers2\Game2' -ArgumentList '-plugins:$PLUG','-appData:C:\SpinProbe-run\appdata','-nosplash','-allowMultiple'" >/dev/null 2>&1
  echo "$label: launched; our PIDs after 20s: $(sleep 20; ours | tr '\n' ' ')"
}
launch; attempt=1
while :; do
  result=timeout; gone=0
  for i in $(seq 1 $(( seconds / 5 + 120 ))); do
    sleep 5
    f=$(ls $B/spinprobe-*.log 2>/dev/null | head -1)
    gl=$(ourlog)
    if [ -n "$f" ] && grep -q "exiting the game" "$f"; then sleep 20; result=done; break; fi
    if [ -n "$gl" ] && grep -q "Preparing Crash Report" "$gl"; then sleep 8; result=crash; break; fi
    if [ $i -gt 6 ] && [ -z "$(ours)" ]; then gone=$((gone+1)); else gone=0; fi
    if [ $gone -ge 3 ]; then result=exited; break; fi
  done
  # Runs 69/71: a launch right after killing the previous instance sometimes died ~1 s into loading definitions.
  # Retry once when the game is gone within 2 minutes and the plugin never reached session configuration.
  if [ $attempt -eq 1 ] && [ "$result" = exited ] && [ $(( $(date +%s) - start )) -lt 120 ] && ! grep -q "ConfigureSession called" $B/spinprobe-*.log 2>/dev/null; then
    echo "$label: died during startup, retrying once"
    attempt=2; rm -f $B/spinprobe-*.log; sleep 5; launch; continue
  fi
  break
done
kill_ours
cp $B/spinprobe-*.log /d/SpinProbe/runs/$label.log 2>/dev/null
gl=$(ourlog)
[ -n "$gl" ] && cp "$gl" /d/SpinProbe/runs/$label.game.log 2>/dev/null
echo "$label: $result after $(( $(date +%s) - start ))s"
