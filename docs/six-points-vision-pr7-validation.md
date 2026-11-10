# PR #7 six-point vision integration

The six-point geometry strategy now lives at `code/vision/dachuang_strategy.py` and `code/vision/dachuang/`. The normal `VIS` workflow consumes source landmarks P5/P6 as its local points 1/2. `VIS_HEAD` consumes source landmarks P1–P4 in the same order. Both continue through the existing `vision_workflows.py` selection and target-calculation path.

Calibration stays under `config/`, resolved by `settings.py`: `xy_pixel_calibration.json` and `manual_left_p2_20260909.json`. New compensation files live under `config/` and are selected by the matching workflow. For head P4, the new correction `(X=0, Y=80)` differs from the legacy `(-38, +106)` by `(+38, -26)` steps; the old table is not used for new landmarks.

The latest target `main` was merged before the integration. `motion_client.handle_motion_command`, `serial_motion.vision_reference_ready`/`motion_fault`, and the drill console flow remain present. The abdomen XY displacement guard is 1200 steps, matching the user's previous setting for the new targets.

Validation passed in a clean virtual environment: the drill console and new vision/config modules import, all 54 tests pass, and `cockroach3.jpg`, `cockroach4.jpg`, and `cockroach5.jpg` return six landmarks in the expected P1–P6 order. VIS selects source P5/P6 and VIS_HEAD selects P1–P4; the corresponding compensation tables use workflow-local point numbers. These samples are marked `GEOMETRY ESTIMATE - REVIEW`, so inspect the preview before using estimated positions. No live camera or machine motion was tested; hardware verification remains not run.
