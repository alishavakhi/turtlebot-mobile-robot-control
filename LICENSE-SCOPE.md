# Licence scope

`LICENSE` is a plain MIT licence and covers the files in this repository that
the author wrote:

* `src/matlab/` — initialisation scripts, `draw_robot.m`, the analysis pipeline
* `src/simulink/` — the five Simulink models, as built by the author
* `src/model_code/` — the MATLAB Function blocks extracted from those models
* `data/` — logs, exports and metric tables produced by the author's runs
* `figures/`, `docs/`, `tools/` and the README
* `report/Ali_Shavakhi_Final_Report.pdf` — the author's submitted report

## What is deliberately not here

Files that came from the course or from MathWorks are **not** redistributed in
this repository, so no licence question arises for them. They are named as
dependencies in `THIRD_PARTY_NOTICES.md` and `docs/SELECTION.md`:

* the laboratory's Vicon interface block (`getViconMeasure_*.slx`);
* the ROS message-bus package that Simulink generates (`+bus_conv_fcns/`);
* lecture slides, the activity briefs and the report template;
* the other students' folders that happened to sit in the same archive.

## Data

The recorded runs are the author's own experimental data, published here for
review and reuse with attribution. If you use them in academic work, cite this
repository. They are provided with no warranty of correctness beyond what
`tools/verify_metrics.py` checks.
