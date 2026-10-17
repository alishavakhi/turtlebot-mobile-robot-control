# Test plan and test results

Two different things are reported here, kept deliberately separate:

* **what this repository proves** — checks that were executed, with their output;
* **what the original work demonstrated** — the runs the author performed in the
  laboratory, evidenced by the logs and the report, and *not* re-executed here.

Nothing in this file claims a test that was not run on the machine that prepared
the repository.

## Environment of the checks below

```text
OS        Linux (no MATLAB, no Simulink, no ROS installed, none attempted)
Python    3.12 (venv), numpy 2.5.4, scipy 1.18.1
MATLAB    not available — every .slx here was read as a zip/XML package, never executed
```

## 1. Metric recomputation — `tools/verify_metrics.py`

Purpose: prove that the numbers in `docs/RESULTS.md` and in the report come out
of the recorded data. MATLAB is not needed: the analysis pipeline had already
exported the channels to plain MAT structs and CSVs.

```text
$ python tools/verify_metrics.py
28/28 checks passed
```

What is recomputed, per task:

| Task | Recomputations |
|---|---|
| 1 (real, 3 runs) | final Vicon position and heading error for each run; peak wheel command against the 10 rad/s limit; agreement with the summary table; that all three runs met the 0.05 m position threshold and none met the heading threshold |
| 2 (real) | localization RMSE overall, maximum error, RMSE restricted to the noisy (red) rectangle, final position/heading error at the goal, peak commanded wheel speed, that this run had 0/509 lost measurements |
| 3 (real) | EKF RMSE over the samples with a valid Vicon fix *and* over those without (recomputed from the pose arrays), share of the run without a fix, final distance to the target pose, peak wheel command, and that the stored metric field equals the printed value |
| 4 (simulation) | only that the five exports exist and are opaque `Simulink.SimulationOutput` objects — the run data cannot be read outside MATLAB, and this is stated rather than worked around |

Integrity checks inside the same script:

* every published file matches `docs/PROVENANCE.sha256`;
* re-extracting the MATLAB Function blocks from every model reproduces
  `src/model_code/` byte for byte;
* every `.slx` is a readable Simulink package.

## 2. Repository guards — `tools/static_checks.py`

Purpose: catch the mistakes that make a repository unsafe rather than wrong.

```text
$ python tools/static_checks.py
13/13 static checks passed
```

Checks: no credential-shaped strings (matched structurally, never by a literal
value); no email addresses; no student-ID field; no private IP addresses; no
ROS master URI or lab setup notes; no course, peer or generated material in the
tree; the only PDF is the submitted report; every README image exists, is a
non-trivial PNG, and at least four are embedded; every relative link resolves;
no text file carries the 0x1A corruption the originals had; and every decimal in
a **verified** table of `docs/RESULTS.md` traces to the metric table named above
it (26 values checked across 5 tables).

## 3. Documentation is generated, not typed — `tools/model_inventory.py`

```text
$ python tools/model_inventory.py --check
docs/MODEL_STRUCTURE.md is up to date
```

The block inventory is read out of the `.slx` files, so a stale document fails
this check instead of quietly lying.

## 4. What was NOT executed, and why

| Not executed | Why | Consequence |
|---|---|---|
| MATLAB / Simulink simulation of any task | MATLAB is not installed on the preparation host, and it is a licensed product that cannot be substituted | The models were not run here. Their structure and their embedded code were read directly instead. |
| Octave substitution | Octave has no Simulink, and its Control/`acker`/`lqr` behaviour differs from MATLAB's | Deliberately not attempted — a passing Octave run would not have been evidence about these models |
| ROS node build / real-robot run | needs the lab TurtleBot, the Vicon interface block and the laboratory ROS master | Hardware reproduction is impossible off-campus; the logs of the original runs are included instead |
| Task 4 metric recomputation | the exports contain MATLAB class objects (MCOS) that SciPy cannot decode | Quoted from the report and labelled as not recomputed |

## 5. What the original work demonstrated (author's runs, not re-executed)

Evidenced by `data/real/` and by `report/Ali_Shavakhi_Final_Report.pdf`:

* three real posture-regulation runs, each reaching the position threshold;
* one real navigation run through the noisy-Vicon region;
* one real circular-tracking run inside the 90 % Vicon-loss region;
* two Task 4 simulations, one from an exact and one from a perturbed initial pose.

These are results of the original implementation in the original environment.
The reconstruction above verifies their *numbers*; it does not re-run the system.
