## Trajectory Format

The output trajectory's format is the same as TUM trajectory format under NED coordinate, with 8 columns, each representing

```
timestamp tx ty tz qx qy qz qw
```

## Run Evaluation

To run evaluation, use `evaluate.py` file in this directory with following CLI arguments:

```
python ./evaluate.py --estimate <airdos_output_path> --gt <tartanair_shibuya_path> [--real_perspective]?
```

The `--real_perspective` flag is optional. Adding this flag will plot the resulted trajectory while enforcing the 1:1 perspective ratio.
