from evo.tools.file_interface import read_tum_trajectory_file
from evo.main_ape import ape
from evo.main_rpe import rpe
from evo.core.metrics import PoseRelation, Unit
from evo.tools.plot import traj, PlotMode
from evo.tools.settings import SETTINGS
from pathlib import Path

import argparse
import matplotlib.pyplot as plt

Parser = argparse.ArgumentParser(description="Plot and evaluate the AirDOS output trajectory on TartanAir-shibuya Dataset")
Parser.add_argument("--estimate", action="store", required=True)
Parser.add_argument("--gt", action="store", required=True)
Parser.add_argument("--real_perspective", action="store_true", default=False)


args = Parser.parse_args()
est_traj_path = Path(args.estimate)
gt_traj_path = Path(args.gt)

assert est_traj_path.exists()
assert gt_traj_path.exists()

SETTINGS.plot_xyz_realistic = args.real_perspective
est_traj = read_tum_trajectory_file(est_traj_path)
gt_traj = read_tum_trajectory_file(gt_traj_path)

ape_output = ape(gt_traj, est_traj, pose_relation=PoseRelation.translation_part, align=True, correct_scale=False)
rpe_t_output = rpe(gt_traj, est_traj, pose_relation=PoseRelation.translation_part, delta=1.0, delta_unit=Unit.frames,
                 all_pairs=True, align=True, correct_scale=False)
rpe_r_output = rpe(gt_traj, est_traj, pose_relation=PoseRelation.rotation_part, delta=1.0, delta_unit=Unit.frames,
                 all_pairs=True, align=True, correct_scale=False)

print("ATE:", round(ape_output.stats["rmse"], 3))
print("RPE (Translation)", round(rpe_t_output.stats["rmse"], 3))
print("RPE (Rotation)", round(rpe_r_output.stats["rmse"], 3))

fig = plt.figure(dpi=200)
ax = fig.add_subplot(1, 1, 1)
traj(ax, plot_mode=PlotMode.xz, traj=est_traj, color="r", label="AirDOS")
traj(ax, plot_mode=PlotMode.xz, traj=gt_traj, color="b", label="Ground Truth")
plt.show()
