# Selection — what is in this repository, and what is not

The final activity arrived inside a 4 420-file, 245 MiB archive that also
contained the whole course (four numerical activities, two experimental
activities), the lecture slides, the activity briefs, and the working folders of
three other students. This repository is the **final activity only**.

## What was published

Counts are for the tree as published (regenerate with `git ls-files | wc -l`).

| Group | Files | Why |
|---|---|---|
| `src/matlab/` — `fa.m`, `FA_modified.m`, `draw_robot.m` | 3 | the workspace initialisation and the map/robot visualisation the models call |
| `src/matlab/analysis/` | 34 | the pipeline that produced every figure and every metric table in the report |
| `src/simulink/` | 6 | four task simulations, the analytical-Jacobian Task 4 variant, and the real-robot model |
| `src/model_code/` | 60 | the MATLAB Function blocks extracted verbatim from those models |
| `data/real/` | 15 | real-robot logs: raw `SimulationOutput` objects plus the exported plain-struct versions used for analysis |
| `data/simulation/` | 12 | exported simulation runs for all four tasks |
| `data/metrics/` | 11 | the metric tables the report was written from |
| `figures/` | 18 | the figures used in the README and in `docs/RESULTS.md` |
| `report/` | 1 | the submitted 31-page final report |
| `docs/`, `tools/`, root files | 15 | method, results, model structure, test plan, this file, licence and notices |

## What was excluded, and why

| Category | Files | Reason |
|---|---|---|
| Simulink build output (`slprj/`, `*_ert_rtw/`, `+bus_conv_fcns/`) | 2 847 | generated; rebuilds from the model; ~100 MB of binaries |
| `.mat` objects inside build folders | 1 768 | generated caches, not results |
| Byte-identical duplicates (`robotic lab/` vs `Robotic lab materials/`) | 963 | the same tree copied twice |
| `NA1`, `NA2`, `NA3`, `EA1`, `EA2` work | ~120 | preliminary activities, explicitly out of scope |
| `johny/`, `Filippo/`, `Hosam/`, `amir files/` | 798 | other students' work |
| Lecture slides and course notes | 10 PDFs, ~20 MiB | the professor's material |
| Activity briefs and report template | 6 | course material |
| `robotics_oral_exam_prep_ultimate_coverage.pdf` | 1 | an exam-preparation dossier, not engineering evidence |
| `roboic lab_.docx` | 1 | personal notes containing a laboratory-network address |
| Vicon interface models (`getViconMeasure_*.slx`) | 3 | course scaffolding; see `THIRD_PARTY_NOTICES.md` |
| Superseded model versions (`FA_.slx`, 202 blocks vs 290) | 1 | the 2024b model is a superset; the older one adds nothing |
| Alternative Task 4 figure sets (`*_plots/` variants) | 15 PNGs + PDFs | three renderings of the same two runs; the final-report set is kept |

## Decisions that were not obvious

### The Task 4 model name is wrong, and the file had to be read to find out

`task4_simulation.slx` and `task4_simulation_backup_before_analytic_H.slx`
differ in one function block. The file *named* "backup before analytic H"
actually **contains** the analytical Jacobian (`lidarModelAnalytic`,
`rayLineJacobian`), and it is that version whose results the report describes
and whose exports are labelled `*_analytic_jacobian.mat`. Assuming the name was
truthful would have published the model that did *not* produce the reported
numbers. Both variants are shipped, renamed after their content:

* `task4_simulation.slx` — numerical Jacobian (finite differences);
* `task4_simulation_analytic_jacobian.slx` — analytical ray-wall Jacobian, the
  one behind the report.

### Three dated EA1 sessions, one final-activity tree

The final activity exists in three dated copies (`final activity`,
`final activity 18.05.2026`, `final activity 19.05.2026`). The 19.05 copy is the
one whose `fa.m` carries the author's header and the real-robot/simulation mode
switch, and it matches the model set used for the report, so it is the one
published.

### Both real and exported logs are kept

`data/real/raw/*.mat` are the untouched `Simulink.SimulationOutput` objects —
opaque outside MATLAB, but they are the primary record. `data/real/*/**.mat`
are the plain-struct exports the analysis pipeline wrote from them, and they are
what `tools/verify_metrics.py` reads. Keeping both is what makes the numbers
checkable while leaving the originals intact.

### The report PDF keeps its original file name

`report/Ali_Shavakhi_Final_Report.pdf` is the submitted document, unmodified
(title, author, 31 pages, 28 figures). Renaming or re-exporting it would break
the link to the submission.

## What is not claimed

* Nothing here is the whole course. Four numerical activities and two
  experimental activities were completed in the same semester and are not part
  of this repository.
* No MATLAB or Simulink execution happened where this repository was prepared;
  see `docs/TEST_PLAN.md` for exactly what was and was not run.
