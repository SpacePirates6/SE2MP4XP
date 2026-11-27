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
audit_client=%s
" "${5:-0.02}" "$seconds" "$vanilla" "${4:-0}" "${6:-always}" "${7:-kinematic}" "${8:-0}" "${9:-Welcome To Verdure 1 47}" "${10:-0.5}" "${11:-0}" > $B/spinprobe.ini
# Extra settings: EXTRA_INI holds ini lines separated by a literal backslash-n (see the run notes).
[ -n "$EXTRA_INI" ] && printf '%b\n' "$EXTRA_INI" >> $B/spinprobe.ini
start=$(date +%s)
# Built from quoted pieces: inside double quotes a backslash before $label escapes it (runs 18/19 loaded no plugin).
PLUG='D:\SpinProbe\live\'"$label"'\ChildlessParents.dll'
# Launch THROUGH STEAM like the user's normal games (no -allowMultiple, no direct exe start).
"/c/Program Files (x86)/Steam/steam.exe" -applaunch 1133870 "-plugins:$PLUG" "-appData:C:\SpinProbe-run\appdata" -nosplash &
echo "$label: launched; our PIDs after 20s: $(sleep 20; ours | tr '\n' ' ')"
result=timeout; gone=0
for i in $(seq 1 $(( seconds / 5 + 120 ))); do
  sleep 5
  f=$(ls $B/spinprobe-*.log 2>/dev/null | head -1)
  gl=$(ls -t $G/SpaceEngineers2_*.log 2>/dev/null | grep -v -E "_(Mission|Render12|Stats)" | head -1)
  if [ -n "$f" ] && grep -q "exiting the game" "$f"; then sleep 20; result=done; break; fi
  if [ -n "$gl" ] && [ "$(stat -c %Y "$gl")" -gt "$start" ] && grep -q "Preparing Crash Report" "$gl"; then sleep 8; result=crash; break; fi
  if [ $i -gt 6 ] && [ -z "$(ours)" ]; then gone=$((gone+1)); else gone=0; fi
  if [ $gone -ge 3 ]; then result=exited; break; fi
done
kill_ours
cp $B/spinprobe-*.log /d/SpinProbe/runs/$label.log 2>/dev/null
gl=$(ls -t $G/SpaceEngineers2_*.log 2>/dev/null | grep -v -E "_(Mission|Render12|Stats)" | head -1)
cp "$gl" /d/SpinProbe/runs/$label.game.log 2>/dev/null
echo "$label: $result after $(( $(date +%s) - start ))s"
