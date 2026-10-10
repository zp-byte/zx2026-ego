#!/usr/bin/env bash
for i in $(seq 1 70); do
  if grep -q 'FLOCK RUN DONE' /tmp/r1_run.log 2>/dev/null; then
    break
  fi
  sleep 5
done
echo "--- final log (tail) ---"
tail -n 120 /tmp/r1_run.log
