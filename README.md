# MCG4366 Capstone Parameterization

## How To Run The Code
1. Open MATLAB.
2. Open and run `MATLAB/Main.m`.
3. In the app, enter patient body weight and height, then click **Generate**.
4. The app will compute dimensions, update the table/plots/log tab, and write output files.

## Folder Structure
- `Log/`
  - Contains the generated analysis log file: `group12_LOG.txt`.

- `MATLAB/`
  - Main MATLAB project code.
  - `Main.m`: entry point that launches the GUI and runs the full analysis pipeline.
  - `Parameterization.prj`: MATLAB project file.
  - `analysis/`: analysis modules (joint reaction force, shaft/frame/hydraulic/lock calculations, plots).
  - `components/`: component-level sizing and support classes.
  - `data/`: input CSV datasets used by the analysis.
  - `gui/`: App Designer app and GUI launch helper.
  - `logs/`: internal analysis logs such as `JRFLogFile.txt`.
  - `resources/project/`: MATLAB project metadata files.

- `Solidworks/`
  - `Equations/`: text equation files used to parameterize SolidWorks parts.

## What Gets Populated
- `Solidworks/Equations/`
  - Populated/updated with generated dimension files after running the analysis, including:
  - `ball.txt`, `ecase_rear.txt`, `ecase_side.txt`, `frame.txt`, `journalbearing.txt`, `key.txt`,
    `lock_handle.txt`, `lock_latch.txt`, `lock_pin.txt`, `lock_pin_screw.txt`,
    `lock_spring.txt`, `lock_spring_screw.txt`, `retainingring.txt`, `shaft.txt`.

- `Log/`
  - `group12_LOG.txt` is generated/updated with the full run log.

- `MATLAB/logs/`
  - `JRFLogFile.txt` is generated/updated by the joint reaction force analysis.

## Purpose
Parameterize our design in SolidWorks by taking in the user height and weight.
