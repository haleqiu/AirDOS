from Util import *

import os.path
import numpy as np
import matplotlib.pyplot as plt

def plot_trajectory_views(fileName: str, *args: List[List[Pose]]):
    view_z = plt.figure(figsize=(12, 12), dpi=400)
    view_x = plt.figure(figsize=(12, 12), dpi=400)
    view_y = plt.figure(figsize=(12, 12), dpi=400)
    ax_z = view_z.add_subplot(1, 1, 1)
    ax_x = view_x.add_subplot(1, 1, 1)
    ax_y = view_y.add_subplot(1, 1, 1)
    ax_z.set_xlabel("x"); ax_z.set_ylabel("y")
    ax_x.set_xlabel("y"); ax_x.set_ylabel("z")
    ax_y.set_xlabel("x"); ax_y.set_ylabel("z")

    min_v, max_v = 1000, -1000
    for label, trajectory in args:
        coord = SE3_to_heterogeneous(trajectory)
        max_v = max(max_v, np.max(coord))
        min_v = min(min_v, np.min(coord))
        x, y, z = coord[:, 0], coord[:, 1], coord[:, 2]
        ax_z.plot(x, y, label=label); ax_x.plot(y, z, label=label); ax_y.plot(x, z, label=label)
    
    ax_x.set_xlim(min_v, max_v); ax_x.set_ylim(min_v, max_v)
    ax_y.set_xlim(min_v, max_v); ax_y.set_ylim(min_v, max_v)
    ax_z.set_xlim(min_v, max_v); ax_z.set_ylim(min_v, max_v)
    ax_x.legend(); ax_y.legend(); ax_z.legend()
    view_x.savefig(fileName + "_yz.jpg")
    view_y.savefig(fileName + "_xz.jpg")
    view_z.savefig(fileName + "_xy.jpg")
    plt.close(view_x); plt.close(view_y); plt.close(view_z)

# Set data source and plotting tasks
plot_tasks = [
    {"name" : "RoadCrossing03",
     "path" : "./data",
     "cvt"  : True,
     "gt"   : "gt_RoadCrossing03.txt",
     "res"  : [ "1_result_RoadCrossing03.txt",
                "2_result_RoadCrossing03.txt",
                "3_result_RoadCrossing03.txt",
                "4_result_RoadCrossing03.txt",
                "5_result_RoadCrossing03.txt"]},
    {"name" : "RoadCrossing04",
     "path" : "./data",
     "cvt"  : True,
     "gt"   : "gt_RoadCrossing04.txt",
     "res"  : [ "1_result_RoadCrossing04.txt",
                "2_result_RoadCrossing04.txt",
                "3_result_RoadCrossing04.txt",
                "4_result_RoadCrossing04.txt",
                "5_result_RoadCrossing04.txt"]},
    {"name" : "RoadCrossing05",
     "path" : "./data",
     "cvt"  : True,
     "gt"   : "gt_RoadCrossing05.txt",
     "res"  : [ "1_result_RoadCrossing05.txt",
                "2_result_RoadCrossing05.txt",
                "3_result_RoadCrossing05.txt",
                "4_result_RoadCrossing05.txt",
                "5_result_RoadCrossing05.txt"]},
    {"name" : "RoadCrossing06",
     "path" : "./data",
     "cvt"  : False,
     "gt"   : "gt_RoadCrossing06.txt",
     "res"  : [ "1_result_RoadCrossing06.txt",
                "2_result_RoadCrossing06.txt",
                "3_result_RoadCrossing06.txt",
                "4_result_RoadCrossing06.txt",
                "5_result_RoadCrossing06.txt"]},
    {"name" : "RoadCrossing07",
     "path" : "./data",
     "cvt"  : False,
     "gt"   : "gt_RoadCrossing07.txt",
     "res"  : [ "1_result_RoadCrossing07.txt",
                "2_result_RoadCrossing07.txt",
                "3_result_RoadCrossing07.txt",
                "4_result_RoadCrossing07.txt",
                "5_result_RoadCrossing07.txt"]},
    {"name" : "Standing01",
     "path" : "./data",
     "cvt"  : True,
     "gt"   : "gt_Standing01.txt",
     "res"  : [ "2_result_Standing01.txt",
                "3_result_Standing01.txt",
                "4_result_Standing01.txt",
                "5_result_Standing01.txt"]},
    {"name" : "Standing02",
     "path" : "./data",
     "cvt"  : True,
     "gt"   : "gt_Standing02.txt",
     "res"  : [ "2_result_Standing02.txt",
                "3_result_Standing02.txt",
                "4_result_Standing02.txt",
                "5_result_Standing02.txt"]}
]

for task in plot_tasks:
    print(f"Plotting {task['name']} ...", end="", flush=True)
    raw_results       = [np.loadtxt(os.path.join(task["path"], fileName)) for fileName in task["res"]]
    result_poses      = [sequence_to_SE3(raw_data) for raw_data in raw_results]
    aligned_sequences = [align_sequence(seq) for seq in result_poses]
    if task["cvt"]:
        aligned_sequences = [
            (label.split(".")[0], [align_to_gt_frame(p) for p in seq])
            for label, seq in zip(task["res"], aligned_sequences)
        ]
    else:
        aligned_sequences = [
            (label.split(".")[0], seq)
            for label, seq in zip(task["res"], aligned_sequences)
        ]
    
    gt_results        = np.loadtxt(os.path.join(task["path"], task["gt"]))
    gt_sequence       = align_sequence(sequence_to_SE3(gt_results))
    plot_trajectory_views(task["name"], ("Ground Truth", gt_sequence), *aligned_sequences)
    print(f" Finished" + ("[Converted]" if task["cvt"] else ""))

