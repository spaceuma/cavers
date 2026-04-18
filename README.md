# CAVERS dataset

![cavers cover](/cover.png)

[![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/2604.15052)
[![Static Badge](https://img.shields.io/badge/Zenodo-dataset-blue)](https://doi.org/10.5281/zenodo.19367714)
[![Citation](https://img.shields.io/badge/Cite-This%20Work-orange)](#citation)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](requirements.txt)

These scripts (adapted from [BASEPROD Scripts](https://github.com/spaceuma/baseprod/)) were used to prepare the [CAVERS dataset](https://doi.org/10.5281/zenodo.19367714). 
The code is released under the [MIT License](LICENSE).

Authors:

- Giacomo Franchini [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0009-0009-5641-8346)
- David Rodríguez-Martínez [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0000-0003-4817-9225)
- Alfonso Martínez-Petersen [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0009-0000-5117-6231)

Supervisors:

- Carlos Pérez del Pulgar [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0000-0001-5819-8310)
- Marcello Chiaberge [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0000-0002-1921-0126)

🙌 This work was possible thanks to the help and support of Giovanni Mastrorocco, Jesus Juli Fernández, Levin Gerdes, Pedro Cantalejo Duarte and Pedro Cantalejo Espejo.

## Links

- Paper: https://arxiv.org/abs/2604.15052 (preprint version)
- Data: https://doi.org/10.5281/zenodo.19367714

## Replay rosbags

First download and unzip the (relevant parts of the) [dataset](https://doi.org/10.5281/zenodo.19367714) to your local drive.

The dataset has been recorded as rosbags with ROS2 Humble in mcap format. Since some additional messages are needed to fully replay the data, a docker image is provided so you don't have to worry about anything. If you don't want to use docker, please step to the [local installation section](#local-installation).

### Using docker
To install docker, please follow the official [installation guide](https://www.docker.com/).

When creating the container, docker will search for the environment variable `CAVERS` and mount a volume inside `/root/dataset/` pointing to the dataset folder on the host. This allow to access the dataset from inside the container. You can export the variable by running: `export CAVERS=/path/to/cavers`.

First clone the repository:

```bash
git clone https://github.com/spaceuma/cavers.git
```

Then build the docker image:

```bash
cd cavers/docker
docker build -t spaceuma/cavers:latest .
```

Create and start a docker container:

```bash
docker compose up -d
```

Finally, you can attach to the running container with:

```bash
docker exec -it docker-cavers-dataset-1 bash
```


### Local installation

To install ROS2 Humble, please follow the official [installation guide](https://docs.ros.org/en/humble/Installation.html).

Install the additional ROS2 packages:

```bash
sudo apt update && sudo apt-get install ros-humble-rosbag2-storage-mcap ros-humble-realsense2-camera-msgs ros-humble-velodyne-msgs
```

Be sure to source the ROS2 installation:

```bash
source path/to/ros/setup.bash
```

Then you can replay the rosbag, for example:

```bash
ros2 bag play loc_diablo_1_rosbag/loc_diablo_1/ --clock
```

>[!NOTE]
>**Optional:** we provide the raw float temperature matrices from the thermal camera in the rosbags, but their [custom definition](./docker/thermal_camera/msg/TemperatureMatrix.msg) is needed to correctly replay these messages.

First clone the repository:

```bash
git clone https://github.com/spaceuma/cavers.git
```

Then create a ROS2 workspace and build the provided package:

```bash
mkdir -p cavers_ws/src
cp -r cavers/docker/thermal_camera cavers_ws/src
cd cavers_ws && colcon build && source install/local_setup.bash
```

## Export rosbags to raw data format

### Requirements

The code was tested with Python 3.10.12.
Additional Python requirements are listed in `requirements.txt`
and can be installed in a virtual environment as follows:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Activate the environment with `source .venv/bin/activate` or deactivate it by invoking `deactivate`.

### Usage

Download and unzip the (relevant parts of the) [dataset](https://doi.org/10.5281/zenodo.19367714) to your local drive.
If not done yet, we recommend creating a shorthand via `export CAVERS=/path/to/cavers`.
This step is assumed for the following example commands.

>[!Note]
>View detailed script usage information via `python <script> --help`.

### Exporting mcap recordings

You can export all data recordings via:

```zsh
python export_logs.py -i ${CAVERS}/rosbags/ -o ${CAVERS}/raw_data/
```

This will iterate over all roslogs in that directory. Note that this may take a long time since we write to disk quite often to avoid running out of memory.

## Citation

If you use this work, please cite:

> Franchini, G., Rodríguez-Martínez, D., Martínez-Petersen, A., Pérez-del-Pulgar, C. J. & Chiaberge, M. (2026). CAVERS: Multimodal SLAM Data from a Natural Karstic Cave with Ground Truth Motion Capture. arXiv preprint arXiv:2604.15052.

```
@article{franchini2026cavers,
  title={{CAVERS}: Multimodal SLAM Data from a Natural Karstic Cave with Ground Truth Motion Capture},
  author = {Franchini, Giacomo, Rodríguez-Martínez, David, Martínez-Petersen, Alfonso, Pérez del Pulgar, C.J. and Chiaberge, Marcello},
  journal={arXiv preprint arXiv:2604.15052},
  volume={},
  number={},
  doi = {https://doi.org/10.48550/arXiv.2604.15052},
  url = {https://arxiv.org/abs/2604.15052},
  year = {2026}
}
```
