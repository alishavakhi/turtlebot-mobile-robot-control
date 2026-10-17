# Third-party notices and required dependencies

Nothing in this repository is copied from the course, from MathWorks, or from
another student. Everything that is *required to run* but *not redistributed*
is listed here, with what it is and where it comes from.

## 1. Laboratory Vicon interface block

* **Files (not included):** `getViconMeasure_2024.slx`, `getViconMeasure_2024_2025.slx`,
  `getViconMeasure_2024b.slx`
* **What it does:** subscribes to the motion-capture ROS topic and outputs the
  measured pose of the tracked object. It is the only path from the Vicon
  system into the models.
* **Origin:** provided by the Robotics Laboratory (University of Padova, DEI)
  for use on the laboratory setup.
* **Why it is not here:** the redistribution terms are not established, and it
  is course scaffolding rather than the author's work.
* **Consequence:** the real-robot model (`src/simulink/online_robot_2024b.slx`)
  opens and reads, but cannot be run without this block and the lab network.

## 2. Simulink ROS message conversion package

* **Files (not included):** `+bus_conv_fcns/+ros/**` (~200 generated `.m` files)
* **What it does:** converts ROS messages to Simulink buses and back; generated
  automatically by the Simulink ROS Toolbox when the model is configured.
* **Origin:** MathWorks code generation output, regenerated per model build.
* **Why it is not here:** generated scaffolding with MathWorks' own terms, and
  byte-identical in every folder it appears in. Recreate it by opening the
  model in MATLAB with the ROS Toolbox installed.

## 3. ROS / Simulink tooling

* **Simulink ROS Toolbox** — required to open the real-robot model and to
  generate the ROS node. Commercial MathWorks product; not redistributed.
* **ROS Noetic** — required on the machine that talks to the robot. External
  open-source project, not redistributed.
* **MATLAB / Simulink R2024b** — required to open any `.slx` here. The models
  are saved in the 2024b format.

## 4. Course material

* **Lecture slides** (10 PDFs), **activity briefs** (NA1–NA3, EA1, EA2, final
  activity) and the **report template** were in the same archive. They are the
  course's material, not the author's, and are not included. The objectives
  they define are restated in the author's own words in `docs/METHOD.md`.

## 5. Other students' material

* The archive also contained folders belonging to other students
  (`johny/`, `Filippo/`, `Hosam/`) and a bundle marked `amir files/`. None of
  it is included, and no part of the published work is derived from it.

## 6. Tools used to prepare this repository

* Python 3.9+ with **NumPy** and **SciPy** (BSD-3-Clause) — used by the scripts
  in `tools/`. They are dependencies, not vendored code.
* **Git** and **GitHub** — hosting.
