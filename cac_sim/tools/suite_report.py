#!/usr/bin/env python3
"""Summarise tools/run_seed_suite.sh results:  python3 tools/suite_report.py <out_dir>

A seed PASSES only if, by Gazebo ground truth: the mission state machine
reached COMPLETE, every leg it reported as arrived was really in its zone
(legs_gt_verified; '?' for summaries written before that check existed), no
footprint-wall contact, no footprint-boulder contact and no wheel ever inside a
crater bowl -- and, for runs with material accounting, at least one cycle
deposited sand in the berm target and every credited scoop/dump was confirmed
with the ground-truth pose (material_gt_verified).

Material columns are SIMULATED ESTIMATES (cac_sim/material_model.py): litres in
the berm target, cycles ok/started, mission run time (sim seconds) and whether
it fit the time budget. BCP-mass is the rulebook formula applied to the
simulated volume -- an estimate, not a score.
"""
import glob
import json
import os
import sys

out = sys.argv[1]
rows = []
for f in sorted(glob.glob(os.path.join(out, 'seed_*_summary.json')), key=lambda p: int(p.split('_')[-2])):
    s = json.load(open(f))
    legs = s.get('legs_gt_verified')
    run = s.get('run_summary') or {}
    has_mat = bool(run)
    mat_gt = s.get('material_gt_verified')
    cycles_ok = run.get('cycles_ok')
    nav_ok = (s['final_state'] == 'COMPLETE' and legs is not False and not s['wall_contact']
              and not s['boulder_contact'] and s['crater_wheel_samples'] == 0)
    ok = nav_ok and (not has_mat or (bool(cycles_ok) and mat_gt is True))
    reason = s.get('mission_status', {}).get('fail_reason', '') if s['final_state'] != 'COMPLETE' else ''
    if nav_ok and not ok:
        reason = 'no in-target cycle' if not cycles_ok else 'material event not confirmed by ground truth'
    m = run.get('material', {})
    budget = run.get('time_budget_sec')
    rt = run.get('run_time_sec')
    rows.append((s['seed'], 'PASS' if ok else 'FAIL', s['final_state'], s['duration_sec'], s['loc_err_mean'],
                 s['loc_err_max'], s['min_wall_dist'], s['min_boulder_dist'], s['crater_wheel_samples'],
                 s.get('crater_false_cells'), '?' if legs is None else ('yes' if legs else 'NO'),
                 f'{cycles_ok}/{run.get("cycles_started")}' if has_mat else '-',
                 m.get('deposited_in_target_l') if has_mat else None,
                 '-' if mat_gt is None else ('yes' if mat_gt else 'NO'),
                 rt, None if rt is None or budget is None else ('yes' if rt <= budget else 'NO'),
                 run.get('bcp_mass_estimate'), reason[:70]))
hdr = ('seed', 'result', 'final', 'wall_s', 'loc_mean', 'loc_max', 'wall_min', 'boulder_min', 'crater_wheel',
       'false_cells', 'legs_gt', 'cycles_ok', 'berm_L', 'mat_gt', 'run_s', 'in_budget', 'bcp_mass_est', 'reason')
print(' | '.join(hdr))
for r in rows:
    print(' | '.join('-' if v is None else str(v) for v in r))
n = len(rows)
print(f'\n{sum(r[1] == "PASS" for r in rows)}/{n} seeds passed; '
      f'{sum(r[2] == "COMPLETE" for r in rows)}/{n} reached COMPLETE; '
      f'{sum(r[8] > 0 for r in rows)}/{n} had a wheel in a crater; '
      f'{sum(r[6] is not None and r[6] <= 0.02 for r in rows)}/{n} touched a wall.')
berm = [r[12] for r in rows if r[12] is not None]
if berm:
    print(f'SIMULATED sand in the berm target: total {sum(berm):.2f} L over {len(berm)} runs, '
          f'mean {sum(berm) / len(berm):.2f} L per run (estimate, not a measurement).')
