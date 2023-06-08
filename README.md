# AirDOS: Dynamic SLAM benefits from Articulated Objects

[![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-yellow.svg)](./LICENSE)
[![Air Series](https://img.shields.io/badge/collection-Air%20Series-b31b1b)](https://chenwang.site/airseries/)

![tartanair](./doc/AirDOS.gif)


# TartanAir Shibuya Dataset

> You may download the dataset from: https://github.com/haleqiu/tartanair-shibuya

# Instruction

## Dependencies

* Eigen (Tested on v3.3)
* OpenCV (Tested on Opencv 3.4)
* Pangolin (v0.5)

## Build

Run the `./build.sh` in root directory to build third party libraries, AirDOS and construct a binary representation of ORB vocabulary file.

## Run an example script

Currently, the project only supports `Stereo` mode. The `stereo_human` executable in `Examples/Stereo` 

```bash
./Examples/Stereo/stereo_human \
  ./Vocabulary/ORBvoc.txt \
  ./Examples/Stereo/config/tartanair.yaml \
  /.../TartanAir_shibuya/RoadCrossing07 \
  ./Evaluation/data/trajectory_output.txt
```

## Evaluate Trajectory

For evaluation, please check https://github.com/castacks/tartanair_tools.git.

> Note: the TartanAir Tools expect to read the trajectory format with 7 columns, representing the translation and quaternion.
> 
> The trajectory exported by the program contains 8 columns, where first column is the timestamp of pose.
> 
> To evaluate the exported trajectory using TartanAir Tools, you need to remove the first column of exported trajectory file.

We also provided an evaluation tool based on `evo` package, for more detail, please check this [README.md](Evaluation/README.md).

# Publications

[AirDOS: Dynamic SLAM benefits from Articulated Objects
](https://ieeexplore.ieee.org/document/9811667)

```bibtex
@inproceedings{qiu2022airdos,
  author={Qiu, Yuheng and Wang, Chen and Wang, Wenshan and Henein, Mina and Scherer, Sebastian},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={AirDOS: Dynamic SLAM benefits from Articulated Objects}, 
  year={2022},
  pages={8047-8053},
  doi={10.1109/ICRA46639.2022.9811667}
 }
```

## Contributors

[haleqiu](https://github.com/haleqiu), [MarkChenYutian](https://github.com/MarkChenYutian)
