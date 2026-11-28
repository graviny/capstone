#!/bin/bash
# Webots on the days30 test scenarios (logs/days30/test, 9 files), one run per scenario and method,
# methods of one scenario in parallel. Protocol: docs/paper/baselines_days30.md
#
#   bash tools/webots_days30.sh <method> [<method> ...]
#     S           team WES        (WES_SCHEDULER=sched_verify)
#     H           Hungarian       (WES_SCHEDULER=sched_hungarian)
#     mod:<name>  any scheduler module lib/<name>.py
#     mod:<name>:<model file>  the same, with ASSIGN_WEIGHTS=<model file> (one per training run r0, r1, r2)
#     <stem>      sched_learned with models/assign_weights_<stem>.json (e.g. days30_cem_r0)
#   env WEBOTS=<webots.exe> to override the path below.
# Overwrites the tracked scenario.json and worlds/warehouse.wbt while it runs and git-restores them
# at the end. Do not open Webots or run gen_world.py meanwhile.
cd "$(dirname "$0")/.." || exit 1
WB=${WEBOTS:-/d/Webots/msys64/mingw64/bin/webots.exe}
OPT="--batch --mode=fast --minimize --no-rendering --stdout --stderr"
METHODS="$*"
[ -n "$METHODS" ] || { echo "usage: $0 <method> ..."; exit 1; }
rm -f logs/_supervisor_crash.txt
for f in logs/days30/test/*.json; do
  read d s < <(python -c "import json,sys;c=json.load(open(sys.argv[1],encoding='utf-8'));print(c['outbound']['date'],c['seed'])" "$f")
  chmod +w scenario.json worlds/warehouse.wbt
  python tools/gen_world.py --plain --date $d --seed $s > /dev/null || exit 1
  python -c "import json,sys;c=json.load(open(sys.argv[1],encoding='utf-8'));c['run_name']='d30';open('scenario.json','w',encoding='utf-8').write(json.dumps(c,indent=1))" "$f"
  echo "$(date +%H:%M) start $(basename $f) ($METHODS)"
  port=1371
  for m in $METHODS; do
    base="QUIT_AT_END=1 WES_MODULE=wes_runner VERIFY_POLICY=event"
    case $m in
      S) env $base WES_SCHEDULER=sched_verify $WB $OPT --port=$port worlds/warehouse.wbt > /dev/null 2>&1 & ;;
      H) env $base WES_SCHEDULER=sched_hungarian $WB $OPT --port=$port worlds/warehouse.wbt > /dev/null 2>&1 & ;;
      mod:*:*) mm=${m#mod:}; env $base WES_SCHEDULER=${mm%%:*} ASSIGN_WEIGHTS="$(cygpath -w "$PWD/${mm#*:}")" \
           $WB $OPT --port=$port worlds/warehouse.wbt > /dev/null 2>&1 & ;;
      mod:*) env $base WES_SCHEDULER=${m#mod:} $WB $OPT --port=$port worlds/warehouse.wbt > /dev/null 2>&1 & ;;
      *) env $base WES_SCHEDULER=sched_learned ASSIGN_WEIGHTS="$(cygpath -w "$PWD/models/assign_weights_$m.json")" \
           $WB $OPT --port=$port worlds/warehouse.wbt > /dev/null 2>&1 & ;;
    esac
    port=$((port+1))
  done
  wait
  [ -f logs/_supervisor_crash.txt ] && { echo "SUPERVISOR CRASH"; cat logs/_supervisor_crash.txt; exit 1; }
done
chmod +w scenario.json worlds/warehouse.wbt; git checkout -- scenario.json worlds/warehouse.wbt && echo restored
echo "$(date +%H:%M) done"
