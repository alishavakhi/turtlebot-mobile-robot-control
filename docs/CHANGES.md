# Changes made while preparing this repository

The original files were never modified: the intake copy is hash-frozen and
`docs/PROVENANCE.sha256` proves that every published file is byte-identical to
it, except the two files listed in §1, which are covered by the proof below.

## 1. Comment-only repair in `src/matlab/fa.m` and `src/matlab/FA_modified.m`

Both files contained lines whose comment text had been replaced by `0x1A`
(SUB) control bytes before the archive was handed over — the original Persian
comments were already unrecoverable in every encoding (cp1256, cp1252,
latin-1, utf-8 all decode to the same control characters).

Five lines were affected in each file. The inline comments were rewritten in
English where the meaning is unambiguous from the code itself
(`v_max = 0.22` → "linear velocity limit, real-robot mode"), and one line whose
meaning was not recoverable is marked:

```matlab
% [comment removed: the original Persian text was stored as 0x1A control bytes in the source file]
```

**Proof that no code changed:** comments were stripped from both the original
(intake, hash-frozen) and the published file and the remaining code lines
compared — 28 code lines in `fa.m` and 52 in `FA_modified.m`, identical in both
cases. The comparison was run against the frozen intake during preparation; the
intake itself is not part of this repository, so the published code and the
original are stated to be equal rather than re-provable from the repo alone.

The same corruption existed in `draw_robot.m`; there the Persian comments were
intact and are published unchanged.

## 2. File and directory renames

Names were chosen so that a reader can tell what a file is without opening it.
Nothing was renamed to hide its origin: the report keeps its submitted name, and
`docs/SELECTION.md` records where every file came from.

| Original | Published |
|---|---|
| `report for final activity/task1/plot_task1_real_vs_simulation.m` | `src/matlab/analysis/task1_plot_real_vs_simulation.m` |
| `report for final activity/task4/plot_task4_exact_vs_wrong_ekf_analyticH.m` | `src/matlab/analysis/task4_plot_analytic_jacobian.m` |
| `simulink tests/task1/…` (7 legacy scripts) | `src/matlab/analysis/legacy_task1_*.m` |
| `out.simulink.exact_analyticH.mat` | `data/simulation/task4_exact_analytic_jacobian.mat` |
| `task4_simulation_backup_before_analytic_H.slx` | `src/simulink/task4_simulation_analytic_jacobian.slx` (see §3) |
| `FA_2024b.slx` | `src/simulink/online_robot_2024b.slx` |

The single Model-verification consequence: any MATLAB script that loads
`out.simulink.exact_analyticH.mat` by name will need its path updated. The
scripts that read `data/` are the ones in `src/matlab/analysis/`, and they were
written against the original relative layout; they therefore expect the
workspace to be set up as in the report (`init` → run → export), not the layout
of this repository. This is stated in `README.md` §Installation.

## 3. The Task 4 model names were swapped to match their contents

`task4_simulation_backup_before_analytic_H.slx` contains the **analytical**
Jacobian implementation and is the model behind the reported Task 4 results;
`task4_simulation.slx` contains the **numerical** one. Verified by extracting
both models' function blocks: the first calls `lidarModelAnalytic` /
`rayLineJacobian`, the second calls `numericalLidarJacobian`. Both are shipped,
renamed after their content. Details in `docs/SELECTION.md` and
`docs/METHOD.md` §5.2.

## 4. New files written for this repository

Everything below is new work; none of it exists in the original archive:

* `README.md`, `docs/METHOD.md`, `docs/RESULTS.md`, `docs/SELECTION.md`,
  `docs/TEST_PLAN.md`, `docs/CHANGES.md`, `LICENSE`, `LICENSE-SCOPE.md`,
  `THIRD_PARTY_NOTICES.md`, `.gitignore`, `requirements.txt`;
* `tools/extract_model_code.py` — writes the MATLAB Function blocks out of a
  `.slx` as plain `.m`;
* `tools/model_inventory.py` — generates `docs/MODEL_STRUCTURE.md` from the
  models;
* `tools/verify_metrics.py` — recomputes the reported metrics from the logs;
* `tools/static_checks.py` — repository guards;
* `tools/export_metric_tables.py` — exports the two Task 3 quantities that only
  existed inside the MAT struct, so they are checkable like the rest;
* `docs/PROVENANCE.sha256` — the hash of every published file.

## 5. Nothing was deleted from the originals

The archive is intact and read-only at the preparation workspace. The published
subset was copied out of it; the excluded material stayed where it was.
