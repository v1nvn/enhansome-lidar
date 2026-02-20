# Awesome LIDAR [![Awesome](https://awesome.re/badge-flat.svg)](https://awesome.re) with stars

<img src="img/lidar02.svg" align="right" width="200" alt="LIDAR" />

> A curated list of awesome LIDAR sensors and its applications.

[LIDAR](https://en.wikipedia.org/wiki/Lidar) is a remote sensing sensor that uses laser light to measure the surroundings in \~cm accuracy. The sensory data is usually referred as point cloud which means set of data points in 3D or 2D. The list contains hardwares, datasets, point cloud-processing algorithms, point cloud frameworks, simulators etc.

Contributions are welcome! Please [check out](origin/contributing.md) our guidelines.

> \[!TIP]
> An optional view: [szenergy.github.io/awesome-lidar](https://szenergy.github.io/awesome-lidar/)
>
> Source code: [github.com/szenergy/awesome-lidar](https://github.com/szenergy/awesome-lidar) ⭐ 1,226 | 🐛 0 | 📅 2026-01-22

## Contents

* [Awesome LIDAR ](#awesome-lidar-)
  * [Contents](#contents)
  * [Conventions](#conventions)
  * [Manufacturers](#manufacturers)
  * [Datasets](#datasets)
  * [Libraries](#libraries)
  * [Frameworks](#frameworks)
  * [Algorithms](#algorithms)
    * [Basic matching algorithms](#basic-matching-algorithms)
    * [Semantic segmentation](#semantic-segmentation)
    * [Ground segmentation](#ground-segmentation)
    * [Simultaneous localization and mapping SLAM and LIDAR-based odometry and or mapping LOAM](#simultaneous-localization-and-mapping-slam-and-lidar-based-odometry-and-or-mapping-loam)
    * [Object detection and object tracking](#object-detection-and-object-tracking)
    * [LIDAR-other-sensor calibration](#lidar-other-sensor-calibration)
  * [Simulators](#simulators)
  * [Related awesome](#related-awesome)
  * [Others](#others)

## Conventions

* Any list item with ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube) badge has YouTube videos or channel
* Any list item with ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar) badge has a scientific paper or detailed description
* Any list item with ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros) badge is [`ROS 2`](https://docs.ros.org/) compatible
* Any list item with ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github) badge has a GitHub repo or organization, the star count badge ![](https://img.shields.io/github/stars/szenergy/awesome-lidar?color=yellow\&style=flat-square\&logo=github) shows the popularity of the repository

## Manufacturers

* [Velodyne](https://velodynelidar.com/) - Ouster and Velodyne announced the successful completion of their *merger* of equals, effective February 10, 2023. Velodyne was a mechanical and solid-state LIDAR manufacturer. The headquarter is in San Jose, California, USA.
  * [ROS driver ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ros-drivers/velodyne) ⭐ 710 | 🐛 102 | 🌐 C++ | 📅 2025-08-28 ![](https://img.shields.io/github/stars/ros-drivers/velodyne?color=yellow\&style=flat-square\&logo=github)
  * [C++/Python library ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/valgur/velodyne_decoder) ⭐ 51 | 🐛 1 | 🌐 C++ | 📅 2024-12-22 ![](https://img.shields.io/github/stars/valgur/velodyne_decoder?color=yellow\&style=flat-square\&logo=github)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/VelodyneLiDAR)
* [Ouster](https://ouster.com/) - LIDAR manufacturer, specializing in digital-spinning LiDARs. Ouster is headquartered in San Francisco, USA.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/Ouster-lidar)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ouster-lidar) ![](https://img.shields.io/github/stars/ouster-lidar?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Livox](https://www.livoxtech.com/) - LIDAR manufacturer.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCnLpB5QxlQUexi40vM12mNQ)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Livox-SDK) ![](https://img.shields.io/github/stars/Livox-SDK?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [SICK](https://www.sick.com/ag/en/) - Sensor and automation manufacturer, the headquarter is located in Waldkirch, Germany.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/SICKSensors)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/SICKAG) ![](https://img.shields.io/github/stars/SICKAG?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Hokuyo](https://www.hokuyo-aut.jp/) - Sensor and automation manufacturer, headquartered in Osaka, Japan.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCYzJXC82IEy-h-io2REin5g)
* [Pioneer](http://autonomousdriving.pioneer/en/3d-lidar/) - LIDAR manufacturer, specializing in MEMS mirror-based raster scanning LiDARs (3D-LiDAR). Pioneer is headquartered in Tokyo, Japan.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/PioneerCorporationPR)
* [Luminar](https://www.luminartech.com/) - LIDAR manufacturer focusing on compact, auto-grade sensors. Luminar is headquartered Palo Alto, California, USA.
  * [Vimeo channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://vimeo.com/luminartech)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/luminartech) ![](https://img.shields.io/github/stars/luminartech?color=yellow\&style=flat-square\&logo=github)
* [Hesai](https://www.hesaitech.com/) - Hesai Technology is a LIDAR manufacturer, founded in Shanghai, China.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCG2_ffm6sdMsK-FX8yOLNYQ/videos)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/HesaiTechnology) ![](https://img.shields.io/github/stars/HesaiTechnology?color=yellow\&style=flat-square\&logo=github)
* [Robosense](http://www.robosense.ai/) - RoboSense (Suteng Innovation Technology Co., Ltd.) is a LIDAR sensor, AI algorithm and IC chipset maufactuirer based in Shenzhen and Beijing (China).
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCYCK8j678N6d_ayWE_8F3rQ)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/RoboSense-LiDAR) ![](https://img.shields.io/github/stars/RoboSense-LiDAR?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [LSLIDAR](https://www.lslidar.com/) - LSLiDAR (Leishen Intelligent System Co., Ltd.) is a LIDAR sensor manufacturer and complete solution provider based in Shenzhen, China.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/@lslidar2015)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Lslidar) ![](https://img.shields.io/github/stars/Lslidar?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Ibeo](https://www.ibeo-as.com/) - Ibeo Automotive Systems GmbH is an automotive industry / environmental detection laserscanner / LIDAR manufacturer, based in Hamburg, Germany.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/IbeoAutomotive/)
* [Innoviz](https://innoviz.tech/) - Innoviz technologies / specializes in solid-state LIDARs.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCVc1KFsu2eb20M8pKFwGiFQ)
* [Quanenergy](https://quanergy.com/) - Quanenergy Systems / solid-state and mechanical LIDAR sensors / offers End-to-End solutions in Mapping, Industrial Automation, Transportation and Security. The headquarter is located in Sunnyvale, California, USA.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/QuanergySystems)
* [Cepton](https://www.cepton.com/index.html) - Cepton (Cepton Technologies, Inc.) / pioneers in frictionless, and mirrorless design, self-developed MMT (micro motion technology) lidar technology. The headquarter is located in San Jose, California, USA.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCUgkBZZ1UWWkkXJ5zD6o8QQ)
* [Blickfeld](https://www.blickfeld.com/) - Blickfeld is a solid-state LIDAR manufacturer for autonomous mobility and IoT, based in München, Germany.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/BlickfeldLiDAR)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Blickfeld) ![](https://img.shields.io/github/stars/Blickfeld?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Neuvition](https://www.neuvition.com/) - Neuvition is a solid-state LIDAR manufacturer based in Wujiang, China.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UClFjlekWJo4T5bfzxX0ZW3A)
* [Aeva](https://www.aeva.com/) - Aeva is bringing the next wave of perception technology to all devices for automated driving, consumer electronics, health, industrial robotics and security, Mountain View, California, USA.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/AevaInc)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/aevainc) ![](https://img.shields.io/github/stars/aevainc?color=yellow\&style=flat-square\&logo=github)
* [XenomatiX](https://www.xenomatix.com/) - XenomatiX offers true solid-state lidar sensors based on a multi-beam lasers concept. XenomatiX is headquartered in Leuven, Belgium.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/@XenomatiXTruesolidstatelidar)
* [MicroVision](https://microvision.com/) - A pioneer in MEMS-based laser beam scanning technology, the main focus is on building Automotive grade Lidar sensors, located in Hamburg, Germany.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/mvisvideo)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/MicroVision-Inc) ![](https://img.shields.io/github/stars/MicroVision-Inc?color=yellow\&style=flat-square\&logo=github)
* [PreAct](https://www.preact-tech.com/) - PreAct's mission is to make life safer and more efficient for the automotive industry and beyond. The headquarter is located in Portland, Oregon, USA.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/@PreActTechnologies)
* [Pepperl+Fuchs](https://www.pepperl-fuchs.com/) - Is a global technology company, specialized in innovative automation solutions and sensor technologies, such as LiDAR, based in Mannheim, Germany.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/pepperl-fuchs)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/PepperlFuchsUSA)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PepperlFuchs) ![](https://img.shields.io/github/stars/PepperlFuchs?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Riegl](https://www.riegl.com/) - Riegl is a manufacturer of 3D laser scanning systems, based in Austria.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/@RIEGLLIDAR)
  * [GitHub organization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/riegllms) ![](https://img.shields.io/github/stars/riegllms?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)

## Datasets

* [Ford Dataset](https://avdata.ford.com/) - The dataset is time-stamped and contains raw data from all the sensors, calibration values, pose trajectory, ground truth pose, and 3D maps. The data is Robot Operating System (ROS) compatible.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Ford/AVData) ⭐ 309 | 🐛 12 | 🌐 C++ | 📅 2023-06-11 ![](https://img.shields.io/github/stars/Ford/AVData?color=yellow\&style=flat-square\&logo=github)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2003.07969.pdf)
* [Brno Urban Dataset ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Robotics-BUT/Brno-Urban-Dataset) ⭐ 164 | 🐛 1 | 📅 2021-11-15 ![](https://img.shields.io/github/stars/Robotics-BUT/Brno-Urban-Dataset?color=yellow\&style=flat-square\&logo=github) - Navigation and localisation dataset for self driving cars and autonomous robots in Brno, Czechia.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://ieeexplore.ieee.org/document/9197277)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=wDFePIViwqY)
* [Boreas Dataset](https://www.boreas.utias.utoronto.ca/) - The Boreas dataset was collected by driving a repeated route over the course of 1 year resulting in stark seasonal variations. In total, Boreas contains over 350km of driving data including several sequences with adverse weather conditions such as rain and heavy snow. The Boreas data-taking platform features a unique high-quality sensor suite with a 128-channel Velodyne Alpha Prime lidar, a 360-degree Navtech radar, and accurate ground truth poses obtained from an Applanix POSLV GPS/IMU.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/utiasASRL/pyboreas) ⭐ 113 | 🐛 3 | 🌐 Python | 📅 2026-02-12 ![](https://img.shields.io/github/stars/utiasASRL/pyboreas?color=yellow\&style=flat-square\&logo=github)
  * [Paper 📰](https://arxiv.org/abs/2203.10168)
* [Audi A2D2 Dataset](https://www.a2d2.audi) - The dataset features 2D semantic segmentation, 3D point clouds, 3D bounding boxes, and vehicle bus data.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.a2d2.audi/content/dam/a2d2/dataset/a2d2-audi-autonomous-driving-dataset.pdf)
* [Waymo Open Dataset](https://waymo.com/open/) - The dataset contains independently-generated labels for lidar and camera data, not simply projections.
* [Oxford RobotCar](https://robotcar-dataset.robots.ox.ac.uk/) - The Oxford RobotCar Dataset contains over 100 repetitions of a consistent route through Oxford, UK, captured over a period of over a year.
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/ORIOxfordRoboticsInstitute)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://robotcar-dataset.robots.ox.ac.uk/images/RCD_RTK.pdf)
* [EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/) - This dataset was collected with our robocar (in human driving mode of course), equipped up to eleven heterogeneous sensors, in the downtown (for long-term data) and a suburb (for roundabout data) of Montbéliard in France. The vehicle speed was limited to 50 km/h following the French traffic rules.
* [NuScenes](https://www.nuscenes.org/) - Public large-scale dataset for autonomous driving.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1903.11027.pdf)
* [Lyft](https://level5.lyft.com/dataset/) - Public dataset collected by a fleet of Ford Fusion vehicles equipped with LIDAR and camera.
* [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php) - Widespread public dataset, pirmarily focusing on computer vision applications, but also contains LIDAR point cloud. ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Semantic KITTI](http://semantic-kitti.org/) - Dataset for semantic and panoptic scene segmentation.
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=3qNOXvkpK4I)
* [CADC - Canadian Adverse Driving Conditions Dataset](http://cadcd.uwaterloo.ca/) - Public large-scale dataset for autonomous driving in adverse weather conditions (snowy weather).
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2001.10117.pdf)
* [UofTPed50 Dataset](https://www.autodrive.utoronto.ca/uoftped50) - University of Toronto, aUToronto's self-driving car dataset, which contains GPS/IMU, 3D LIDAR, and Monocular camera data. It can be used for 3D pedestrian detection.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1905.08758.pdf)
* [PandaSet Open Dataset](https://scale.com/open-datasets/pandaset) - Public large-scale dataset for autonomous driving provided by Hesai & Scale. It enables researchers to study challenging urban driving situations using the full sensor suit of a real self-driving-car.
* [Cirrus dataset](https://developer.volvocars.com/open-datasets/cirrus/) A public datatset from non-uniform distribution of LIDAR scanning patterns with emphasis on long range. In this dataset Luminar Hydra LIDAR is used. The dataset is available at the Volvo Cars Innovation Portal.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2012.02938.pdf)
* [USyd Dataset- The Univerisity of Sydney Campus- Dataset](http://its.acfr.usyd.edu.au/datasets/usyd-campus-dataset/) - Long-term, large-scale dataset collected over the period of 1.5 years on a weekly basis over the University of Sydney campus and surrounds. It includes multiple sensor modalities and covers various environmental conditions. ROS compatible
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://ieeexplore.ieee.org/document/9109704)
* [Argoverse ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://www.argoverse.org/) ![](https://img.shields.io/github/stars/argoai/argoverse-api?color=yellow\&style=flat-square\&logo=github) - A dataset designed to support autonomous vehicle perception tasks including 3D tracking and motion forecasting collected in Pittsburgh, Pennsylvania and Miami, Florida, USA.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://openaccess.thecvf.com/content_CVPR_2019/papers/Chang_Argoverse_3D_Tracking_and_Forecasting_With_Rich_Maps_CVPR_2019_paper.pdf)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=DM8jWfi69zM)

## Libraries

* [PyTorch Geometric ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1903.02428.pdf) - A geometric deep learning extension library for PyTorch.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/rusty1s/pytorch_geometric) ⭐ 23,484 | 🐛 1,248 | 🌐 Python | 📅 2026-02-19 ![](https://img.shields.io/github/stars/rusty1s/pytorch_geometric?color=yellow\&style=flat-square\&logo=github)
* [Open3D library](http://www.open3d.org/docs/release/) - Open3D library contanins 3D data processing and visualization algorithms. It is open-source and supports both C++ and Python.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/intel-isl/Open3D) ⭐ 13,329 | 🐛 1,347 | 🌐 C++ | 📅 2026-02-20 ![](https://img.shields.io/github/stars/intel-isl/Open3D?color=yellow\&style=flat-square\&logo=github)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCRJBlASPfPBtPXJSPffJV-w)
* [Point Cloud Library (PCL)](http://www.pointclouds.org/) - Popular highly parallel programming library, with numerous industrial and research use-cases.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PointCloudLibrary/pcl) ⭐ 10,872 | 🐛 593 | 🌐 C++ | 📅 2026-02-05 ![](https://img.shields.io/github/stars/PointCloudLibrary/pcl?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [PyTorch3d](https://pytorch3d.org/) - PyTorch3d is a library for deep learning with 3D data written and maintained by the Facebook AI Research Computer Vision Team.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/facebookresearch/pytorch3d) ⭐ 9,797 | 🐛 304 | 🌐 Python | 📅 2026-01-14 ![](https://img.shields.io/github/stars/facebookresearch/pytorch3d?color=yellow\&style=flat-square\&logo=github)
* [Kaolin](https://kaolin.readthedocs.io/en/latest/) - Kaolin is a PyTorch Library for Accelerating 3D Deep Learning Research written by NVIDIA Technologies for game and application developers.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/NVIDIAGameWorks/kaolin/) ⭐ 5,041 | 🐛 55 | 🌐 Python | 📅 2026-01-22 ![](https://img.shields.io/github/stars/NVIDIAGameWorks/kaolin?color=yellow\&style=flat-square\&logo=github)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1911.05063.pdf)
* [PyVista](https://docs.pyvista.org/) - 3D plotting and mesh analysis through a streamlined interface for the Visualization Toolkit.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/pyvista/pyvista) ⭐ 3,529 | 🐛 741 | 🌐 Python | 📅 2026-02-19 ![](https://img.shields.io/github/stars/pyvista/pyvista?color=yellow\&style=flat-square\&logo=github)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://joss.theoj.org/papers/10.21105/joss.01450)
* [pyntcloud](https://pyntcloud.readthedocs.io/en/latest/) - Pyntcloud is a Python 3 library for working with 3D point clouds leveraging the power of the Python scientific stack.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/daavoo/pyntcloud) ⭐ 1,497 | 🐛 63 | 🌐 Python | 📅 2026-01-28 ![](https://img.shields.io/github/stars/daavoo/pyntcloud?color=yellow\&style=flat-square\&logo=github)
* [LAStools](https://rapidlasso.de/lastools/) - C++ library and command-line tools for pointcloud processing and data compressing.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/LAStools/LAStools) ⭐ 1,037 | 🐛 57 | 🌐 C++ | 📅 2026-01-26 ![](https://img.shields.io/github/stars/LAStools/LAStools?color=yellow\&style=flat-square\&logo=github)
* [pointcloudset](https://virtual-vehicle.github.io/pointcloudset/) - Python library for efficient analysis of large datasets of point clouds recorded over time.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/virtual-vehicle/pointcloudset) ⭐ 51 | 🐛 14 | 🌐 Python | 📅 2025-05-22 ![](https://img.shields.io/github/stars/virtual-vehicle/pointcloudset?color=yellow\&style=flat-square\&logo=github)

## Frameworks

* [Baidu Apollo](https://apollo.auto/) - Apollo is a popular framework which accelerates the development, testing, and deployment of Autonomous Vehicles.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ApolloAuto/apollo) ⭐ 26,434 | 🐛 1,012 | 🌐 C++ | 📅 2026-02-02 ![](https://img.shields.io/github/stars/ApolloAuto/apollo?color=yellow\&style=flat-square\&logo=github)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/ApolloAuto)
* [ALFA Framework ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://ieeexplore.ieee.org/document/11024231) - An open-source framework for developing processing algorithms, with a focus on embedded platforms and hardware acceleration.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github) ![](https://img.shields.io/github/stars/alfa-project/alfa-framework?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)](https://github.com/alfa-project/alfa-framework) ⭐ 18 | 🐛 0 | 🌐 Verilog | 📅 2025-06-16
* [Autoware](https://www.autoware.ai/) - Popular framework in academic and research applications of autonomous vehicles.
  * [GitHub oragnization ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/autowarefoundation) ![](https://img.shields.io/github/stars/autowarefoundation?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.researchgate.net/profile/Takuya_Azumi/publication/327198306_Autoware_on_Board_Enabling_Autonomous_Vehicles_with_Embedded_Systems/links/5c9085da45851564fae6dcd0/Autoware-on-Board-Enabling-Autonomous-Vehicles-with-Embedded-Systems.pdf)

## Algorithms

### Basic matching algorithms

* [KISS-ICP ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=kMMH8rA1ggI) - In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Registration If Done the Right Way.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/kiss-icp) ⭐ 2,071 | 🐛 7 | 🌐 C++ | 📅 2026-01-05 ![](https://img.shields.io/github/stars/PRBonn/kiss-icp?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2209.15397.pdf)
* [Iterative closest point (ICP) ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=uzOCS_gdZuM) - The must-have algorithm for feature matching applications (ICP).
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ethz-asl/libpointmatcher) ⭐ 1,789 | 🐛 99 | 🌐 C++ | 📅 2025-10-29 ![](https://img.shields.io/github/stars/ethz-asl/libpointmatcher?color=yellow\&style=flat-square\&logo=github) - libpointmatcher, a modular library implementing the ICP algorithm.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/pglira/simpleICP) ⭐ 344 | 🐛 17 | 🌐 Python | 📅 2025-01-11 ![](https://img.shields.io/github/stars/pglira/simpleICP?color=yellow\&style=flat-square\&logo=github) - simpleICP C++ /Julia / Matlab / Octave / Python implementation.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://link.springer.com/content/pdf/10.1007/s10514-013-9327-2.pdf) - libpointmatcher: Comparing ICP variants on real-world data sets.
* [Normal distributions transform ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=0YV4a2asb8Y) - More recent massively-parallel approach to feature matching (NDT).

### Semantic segmentation

* [Frustum PointNets ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1711.08488.pdf) - Frustum PointNets for 3D Object Detection from RGB-D Data.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/charlesq34/frustum-pointnets) ⭐ 1,657 | 🐛 90 | 🌐 Python | 📅 2020-03-24 ![](https://img.shields.io/github/stars/charlesq34/frustum-pointnets?color=yellow\&style=flat-square\&logo=github)
* [RandLA-Net ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1911.11236.pdf) - Efficient Semantic Segmentation of Large-Scale Point Clouds
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/QingyongHu/RandLA-Net) ⭐ 1,510 | 🐛 193 | 🌐 Python | 📅 2023-07-11 ![](https://img.shields.io/github/stars/QingyongHu/RandLA-Net?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=Ar3eY_lwzMk)
* [SuperPoint Transformer ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2306.08045.pdf)- Efficient 3D Semantic Segmentation with Superpoint Transformer
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/drprojects/superpoint_transformer) ⭐ 953 | 🐛 2 | 🌐 Python | 📅 2026-02-19 ![](https://img.shields.io/github/stars/drprojects/superpoint_transformer?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=2qKhpQs9gJw)
* [SuperPoint Graph ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1711.09869.pdf)- Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/loicland/superpoint_graph) ⭐ 797 | 🐛 16 | 🌐 Python | 📅 2023-07-19 ![](https://img.shields.io/github/stars/loicland/superpoint_graph?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=Ijr3kGSU_tU)
* [LIDAR-MOS ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.ipb.uni-bonn.de/pdfs/chen2021ral-iros.pdf) - Moving Object Segmentation in 3D LIDAR Data
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/LiDAR-MOS) ⭐ 673 | 🐛 15 | 🌐 Python | 📅 2022-12-21 ![](https://img.shields.io/github/stars/PRBonn/LiDAR-MOS?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=NHvsYhk4dhw)
* [PolarNet ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2003.14032.pdf) - An Improved Grid Representation for Online LiDAR Point Clouds Semantic Segmentation.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/edwardzhou130/PolarSeg) ⭐ 413 | 🐛 27 | 🌐 Python | 📅 2021-05-19 ![](https://img.shields.io/github/stars/edwardzhou130/PolarSeg?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=iIhttRSMqjE)
* [RangeNet++ ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/milioto2019iros.pdf) - Fast and Accurate LiDAR Sematnic Segmentation with fully convolutional network.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/rangenet_lib) ⭐ 347 | 🐛 13 | 🌐 C++ | 📅 2022-07-11 ![](https://img.shields.io/github/stars/PRBonn/rangenet_lib?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=uo3ZuLuFAzk)
* [Automatic labelling ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2108.13757.pdf) - Automatic labelling of urban point clouds using data fusion
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Amsterdam-AI-Team/Urban_PointCloud_Processing) ⭐ 189 | 🐛 4 | 🌐 Python | 📅 2023-02-15 ![](https://img.shields.io/github/stars/Amsterdam-AI-Team/Urban_PointCloud_Processing?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=qMj_WM6D0vI)
* [Study of LIDAR Semantic Segmentation](https://larissa.triess.eu/scan-semseg/) - Scan-based Semantic Segmentation of LiDAR Point Clouds: An Experimental Study IV 2020.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/abs/2004.11803)
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](http://ltriess.github.io/scan-semseg) ![](https://img.shields.io/github/stars/ltriess/scan-semseg?color=yellow\&style=flat-square\&logo=github)

### Ground segmentation

* [LineFit Graph ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://ieeexplore.ieee.org/abstract/document/5548059)- Line fitting-based fast ground segmentation for horizontal 3D LiDAR data
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/lorenwel/linefit_ground_segmentation) ⭐ 784 | 🐛 5 | 🌐 C++ | 📅 2024-07-26 ![](https://img.shields.io/github/stars/lorenwel/linefit_ground_segmentation?color=yellow\&style=flat-square\&logo=github)
* [Patchwork ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2108.05560.pdf)- Region-wise plane fitting-based robust and fast ground segmentation for 3D LiDAR data
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/LimHyungTae/patchwork) ⭐ 576 | 🐛 1 | 🌐 C++ | 📅 2026-02-16 ![](https://img.shields.io/github/stars/LimHyungTae/patchwork?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=rclqeDi4gow)
* [Patchwork++ ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2207.11919.pdf)- Improved version of Patchwork. Patchwork++ provides pybinding as well for deep learning users
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/url-kaist/patchwork-plusplus-ros) ⭐ 418 | 🐛 5 | 🌐 C++ | 📅 2024-05-30 ![](https://img.shields.io/github/stars/url-kaist/patchwork-plusplus-ros?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=fogCM159GRk)
* [Plane Seg ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ori-drs/plane_seg) ⭐ 182 | 🐛 3 | 🌐 C++ | 📅 2021-12-08 ![](https://img.shields.io/github/stars/ori-drs/plane_seg?color=yellow\&style=flat-square\&logo=github) - ROS comapatible ground plane segmentation; a library for fitting planes to LIDAR.
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=YYs4lJ9t-Xo)

### Simultaneous localization and mapping SLAM and LIDAR-based odometry and or mapping LOAM

* [Cartographer ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/cartographer-project/cartographer) ⭐ 7,782 | 🐛 241 | 🌐 C++ | 📅 2024-01-05 ![](https://img.shields.io/github/stars/cartographer-project/cartographer?color=yellow\&style=flat-square\&logo=github) - Cartographer is ROS compatible system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=29Knm-phAyI)
* [LIO-SAM ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2007.00258.pdf) - Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/TixiaoShan/LIO-SAM) ⭐ 4,535 | 🐛 177 | 🌐 C++ | 📅 2025-02-14 ![](https://img.shields.io/github/stars/TixiaoShan/LIO-SAM?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=A0H8CoORZJU)
* [FAST-LIO2 ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2010.08196) - Fast LiDAR-Inertial Odometry is a computationally efficient and robust LiDAR-inertial odometry package
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/hku-mars/FAST_LIO/tree/ROS2) ⭐ 4,323 | 🐛 1 | 🌐 C++ | 📅 2025-01-15 ![](https://img.shields.io/github/stars/hku-mars/FAST_LIO?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=2XNd7P6Qc2s)
* [LeGO-LOAM ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) ⭐ 2,691 | 🐛 40 | 🌐 C++ | 📅 2024-08-17 ![](https://img.shields.io/github/stars/RobustFieldAutonomyLab/LeGO-LOAM?color=yellow\&style=flat-square\&logo=github) - A lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs.
  * ROS 2 verison on different repo: [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/eperdices/LeGO-LOAM-SR) ⭐ 61 | 🐛 2 | 🌐 C++ | 📅 2024-04-11 ![](https://img.shields.io/github/stars/eperdices/LeGO-LOAM-SR?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=7uCxLUs9fwQ)
* [SuMa++ ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf) - LiDAR-based Semantic SLAM.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/semantic_suma/) ⭐ 995 | 🐛 5 | 🌐 C++ | 📅 2024-03-12 ![](https://img.shields.io/github/stars/PRBonn/semantic_suma?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://youtu.be/uo3ZuLuFAzk)
* [MOLA ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://ingmec.ual.es/~jlblanco/papers/EMCEI_2024_Aguilar.pdf) - Modular system for Localization and Mapping, providing LiDAR Odometry (LO), LiDAR-inertial Odometry (LIO), SLAM, localization-only modes, and geo-referencing.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/MOLAorg/mola) ⭐ 867 | 🐛 29 | 🌐 C++ | 📅 2026-02-17 ![](https://img.shields.io/github/stars/MOLAorg/mola?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=sbakEOnsL6Y)
* [OverlapNet ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2020rss.pdf) -  Loop Closing for LiDAR-based SLAM.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/OverlapNet) ⭐ 717 | 🐛 8 | 🌐 Python | 📅 2023-03-24 ![](https://img.shields.io/github/stars/PRBonn/OverlapNet?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=YTfliBco6aw)
* [Removert ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](http://ras.papercept.net/images/temp/IROS/files/0855.pdf) - Remove, then Revert: Static Point cloud Map Construction using Multiresolution Range Images.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/irapkaist/removert) ⭐ 623 | 🐛 17 | 🌐 C++ | 📅 2022-01-19 ![](https://img.shields.io/github/stars/irapkaist/removert?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=M9PEGi5fAq8)
* [KISS-SLAM ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kiss2025iros.pdf) - KISS-SLAM is a simple, robust, and accurate 3D LiDAR SLAM system that just works.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PRBonn/kiss-slam) ⭐ 488 | 🐛 1 | 🌐 Python | 📅 2025-12-05 ![](https://img.shields.io/github/stars/PRBonn/kiss-slam?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [RESPLE ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2504.11580) - Recursive Spline Estimation for LiDAR-Based Odometry
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ASIG-X/RESPLE) ⭐ 212 | 🐛 4 | 🌐 C++ | 📅 2026-01-09 ![](https://img.shields.io/github/stars/ASIG-X/RESPLE?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=3-xLRRT25ys)
* [LOAM J. Zhang and S. Singh ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://youtu.be/8ezyhTAEyHs) - LOAM: Lidar Odometry and Mapping in Real-time.

### Object detection and object tracking

* [urban\_road\_filter ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://doi.org/10.3390/s22010194)-
  Real-Time LIDAR-Based Urban Road and Sidewalk Detection for Autonomous Vehicles
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/jkk-research/urban_road_filter) ⭐ 349 | 🐛 4 | 🌐 C++ | 📅 2025-07-08 ![](https://img.shields.io/github/stars/jkk-research/urban_road_filter?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=T2qi4pldR-E)
* [What You See is What You Get: Exploiting Visibility for 3D Object Detection ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1912.04986.pdf) - By Peiyun Hu, Jason Ziglar, David Held, Deva Ramanan, 2019.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/peiyunh/WYSIWYG) ⭐ 118 | 🐛 2 | 🌐 Python | 📅 2020-10-23 ![](https://img.shields.io/github/stars/peiyunh/WYSIWYG?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=497OF-otY2k)
* [Learning to Optimally Segment Point Clouds ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/abs/1912.04976) - By Peiyun Hu, David Held, and Deva Ramanan at Carnegie Mellon University. IEEE Robotics and Automation Letters, 2020.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/peiyunh/opcseg) ⭐ 25 | 🐛 4 | 🌐 Python | 📅 2021-04-09 ![](https://img.shields.io/github/stars/peiyunh/opcseg?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=wLxIAwIL870)
* [Leveraging Heteroscedastic Aleatoric Uncertainties for Robust Real-Time LiDAR 3D Object Detection ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1809.05590.pdf) - By Di Feng, Lars Rosenbaum, Fabian Timm, Klaus Dietmayer. 30th IEEE Intelligent Vehicles Symposium, 2019.
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=2DzH9COLpkU)
* [detection\_by\_tracker ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://www.semanticscholar.org/paper/3D-LIDAR-Multi-Object-Tracking-for-Autonomous-and-Rachman/bafc8fcdee9b22708491ea1293524ece9e314851) - 3D-LIDAR Multi Object Tracking for Autonomous Driving: Multi-target Detection and Tracking under Urban Road Uncertainties, also used in Autoware Universe
  * [GitHub ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://autowarefoundation.github.io/autoware.universe/main/perception/detection_by_tracker/) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=xSGCpb24dhI)

### LIDAR-other-sensor calibration

* [OpenCalib ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/PJLab-ADG/SensorsCalibration) ⭐ 3,030 | 🐛 145 | 🌐 C++ | 📅 2024-06-17 ![](https://img.shields.io/github/stars/PJLab-ADG/SensorsCalibration?color=yellow\&style=flat-square\&logo=github) - A Multi-sensor Calibration Toolbox for Autonomous Driving
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/2205.14087)
* [direct\_visual\_lidar\_calibration](https://koide3.github.io/direct_visual_lidar_calibration/) - General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/koide3/direct_visual_lidar_calibration) ⭐ 1,330 | 🐛 69 | 🌐 C++ | 📅 2026-01-02 ![](https://img.shields.io/github/stars/koide3/direct_visual_lidar_calibration?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

## Simulators

* [AirSim](https://microsoft.github.io/AirSim) - Unreal Engine based simulator for drones and automotive. Compatible with ROS.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/microsoft/AirSim) ⭐ 17,952 | 🐛 727 | 🌐 C++ | 📅 2025-05-15 ![](https://img.shields.io/github/stars/microsoft/AirSim?color=yellow\&style=flat-square\&logo=github)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=gnz1X3UNM5Y)
* [CARLA](https://carla.org/) - Unreal Engine based simulator for automotive applications. Compatible with Autoware, Baidu Apollo and ROS/ROS 2.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/carla-simulator/carla) ⭐ 13,595 | 🐛 1,168 | 🌐 C++ | 📅 2026-02-19 ![](https://img.shields.io/github/stars/carla-simulator/carla?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UC1llP9ekCwt8nEJzMJBQekg)
* [LGSVL / SVL](https://www.lgsvlsimulator.com/) - Unity Engine based simulator for automotive applications. Compatible with Autoware, Baidu Apollo and ROS/ROS 2. *Note:* LG has made the difficult decision to [suspend](https://www.svlsimulator.com/news/2022-01-20-svl-simulator-sunset) active development of SVL Simulator.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/lgsvl/simulator) ⭐ 2,425 | 🐛 628 | 🌐 C# | 📅 2023-04-04 ![](https://img.shields.io/github/stars/lgsvl/simulator?color=yellow\&style=flat-square\&logo=github)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/c/LGSVLSimulator)
* [OSRF Gazebo](http://gazebosim.org/) - OGRE-based general-purpose robotic simulator, ROS/ROS 2 compatible.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/osrf/gazebo) ⚠️ Archived ![](https://img.shields.io/github/stars/osrf/gazebo?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [AWSIM](https://tier4.github.io/AWSIM) - Unity Engine based simulator for automotive applications. Compatible with Autoware and ROS 2.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/tier4/AWSIM) ⭐ 678 | 🐛 21 | 🌐 C# | 📅 2025-12-06 ![](https://img.shields.io/github/stars/tier4/AWSIM?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=FH7aBWDmSNA)
* [OSSDC SIM](https://github.com/OSSDC/OSSDC-SIM) ⭐ 87 | 🐛 20 | 🌐 C# | 📅 2023-04-10 - Unity Engine based simulator for automotive applications, based on the suspended LGSVL simulator, but an active development. Compatible with Autoware, Baidu Apollo and ROS/ROS 2.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/OSSDC/OSSDC-SIM) ⭐ 87 | 🐛 20 | 🌐 C# | 📅 2023-04-10 ![](https://img.shields.io/github/stars/OSSDC/OSSDC-SIM?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=fU_C38WEwGw)
* [CoppeliaSim](https://www.coppeliarobotics.com/coppeliaSim) - Cross-platform general-purpose robotic simulator (formerly known as V-REP).
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/user/VirtualRobotPlatform)

## Related awesome

* [Awesome machine learning ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/josephmisiti/awesome-machine-learning#readme) ⭐ 71,702 | 🐛 14 | 🌐 Python | 📅 2026-01-29 ![](https://img.shields.io/github/stars/josephmisiti/awesome-machine-learning?color=yellow\&style=flat-square\&logo=github)
* [Awesome deep learning ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/ChristosChristofidis/awesome-deep-learning#readme) ⭐ 27,569 | 🐛 53 | 📅 2025-05-26 ![](https://img.shields.io/github/stars/ChristosChristofidis/awesome-deep-learning?color=yellow\&style=flat-square\&logo=github)
* [Awesome computer vision ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/jbhuang0604/awesome-computer-vision#readme) ⭐ 23,066 | 🐛 81 | 📅 2024-05-17 ![](https://img.shields.io/github/stars/jbhuang0604/awesome-computer-vision?color=yellow\&style=flat-square\&logo=github)
* [Awesome artificial intelligence ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/owainlewis/awesome-artificial-intelligence#readme) ⭐ 12,960 | 🐛 82 | 📅 2025-08-12 ![](https://img.shields.io/github/stars/owainlewis/awesome-artificial-intelligence?color=yellow\&style=flat-square\&logo=github)
* [Awesome reinforcement learning ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/aikorea/awesome-rl/#readme) ⭐ 9,582 | 🐛 8 | 📅 2023-05-25 ![](https://img.shields.io/github/stars/aikorea/awesome-rl?color=yellow\&style=flat-square\&logo=github)
* [Awesome electronics ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/kitspace/awesome-electronics#readme) ⭐ 7,392 | 🐛 11 | 📅 2026-01-05 ![](https://img.shields.io/github/stars/kitspace/awesome-electronics?color=yellow\&style=flat-square\&logo=github)
* [Awesome robotics ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Kiloreux/awesome-robotics#readme) ⭐ 6,146 | 🐛 25 | 📅 2024-09-22 ![](https://img.shields.io/github/stars/Kiloreux/awesome-robotics?color=yellow\&style=flat-square\&logo=github)
* [Awesome point cloud analysis ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Yochengliu/awesome-point-cloud-analysis#readme) ⭐ 4,205 | 🐛 4 | 📅 2023-05-19 ![](https://img.shields.io/github/stars/Yochengliu/awesome-point-cloud-analysis?color=yellow\&style=flat-square\&logo=github)
* [Awesome vehicle security and car hacking ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/jaredthecoder/awesome-vehicle-security#readme) ⭐ 4,119 | 🐛 7 | 📅 2024-12-30 ![](https://img.shields.io/github/stars/jaredthecoder/awesome-vehicle-security?color=yellow\&style=flat-square\&logo=github)
* [Awesome robotics libraries ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/jslee02/awesome-robotics-libraries#readme) ⭐ 2,799 | 🐛 0 | 🌐 Python | 📅 2026-02-16 ![](https://img.shields.io/github/stars/jslee02/awesome-robotics-libraries?color=yellow\&style=flat-square\&logo=github)
* [Awesome ROS 2 ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/fkromer/awesome-ros2#readme) ⚠️ Archived ![](https://img.shields.io/github/stars/fkromer/awesome-ros2?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Awesome SLAM datasets ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/youngguncho/awesome-slam-datasets#readme) ⭐ 1,909 | 🐛 13 | 📅 2024-12-13 ![](https://img.shields.io/github/stars/youngguncho/awesome-slam-datasets?color=yellow\&style=flat-square\&logo=github)
* [Awesome LIDAR ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/szenergy/awesome-lidar) ⭐ 1,226 | 🐛 0 | 📅 2026-01-22 ![](https://img.shields.io/github/stars/szenergy/awesome-lidar?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Awesome LIDAR-Camera calibration ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration) ⭐ 1,208 | 🐛 1 | 📅 2024-08-20 ![](https://img.shields.io/github/stars/Deephome/Awesome-LiDAR-Camera-Calibration?color=yellow\&style=flat-square\&logo=github)
* [Awesome-LiDAR-Visual-SLAM ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/sjtuyinjie/awesome-LiDAR-Visual-SLAM) ⭐ 277 | 🐛 0 | 📅 2025-08-14 ![](https://img.shields.io/github/stars/sjtuyinjie/awesome-LiDAR-Visual-SLAM?color=yellow\&style=flat-square\&logo=github)
* [Awesome LiDAR Place Recognition ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/hogyun2/awesome-lidar-place-recognition) ⭐ 203 | 🐛 0 | 📅 2024-10-02 ![](https://img.shields.io/github/stars/hogyun2/awesome-lidar-place-recognition?color=yellow\&style=flat-square\&logo=github)
* [Awesome-LiDAR-MOS ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/neng-wang/Awesome-LiDAR-MOS) ⭐ 43 | 🐛 0 | 📅 2025-05-06 ![](https://img.shields.io/github/stars/neng-wang/Awesome-LiDAR-MOS?color=yellow\&style=flat-square\&logo=github) Moving Object Segmentation

## Others

* [Rerun](https://rerun.io/) - Rerun is a tool for time-aware multimodal data stack and visualizations.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/rerun-io/rerun) ⭐ 10,212 | 🐛 1,344 | 🌐 Rust | 📅 2026-02-19 ![](https://img.shields.io/github/stars/rerun-io/rerun?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/@rerundotio/videos)
* [MeshLab](https://www.meshlab.net/) - MeshLab is an open source, portable, and extensible system for the processing and editing 3D triangular meshes and pointcloud.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/cnr-isti-vclab/meshlab) ⭐ 5,560 | 🐛 189 | 🌐 C++ | 📅 2026-02-03 ![](https://img.shields.io/github/stars/cnr-isti-vclab/meshlab?color=yellow\&style=flat-square\&logo=github)
* [Semantic Segmentation Editor ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor) ⭐ 1,945 | 🐛 61 | 🌐 JavaScript | 📅 2024-09-18 ![](https://img.shields.io/github/stars/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor?color=yellow\&style=flat-square\&logo=github) - Point cloud and image semantic segmentation editor by Hitachi Automotive And Industry Laboratory, point cloud annotator / labeling.
* [Pcx ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/keijiro/Pcx) ⭐ 1,490 | 🐛 25 | 🌐 C# | 📅 2022-08-23 ![](https://img.shields.io/github/stars/keijiro/Pcx?color=yellow\&style=flat-square\&logo=github) - Point cloud importer/renderer for Unity.
* [Photogrammetry importer ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/SBCV/Blender-Addon-Photogrammetry-Importer) ⭐ 1,257 | 🐛 8 | 🌐 Python | 📅 2026-02-16 ![](https://img.shields.io/github/stars/SBCV/Blender-Addon-Photogrammetry-Importer?color=yellow\&style=flat-square\&logo=github) - Blender addon to import reconstruction results of several libraries.
* [Bpy ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/uhlik/bpy) ⭐ 1,069 | 🐛 0 | 🌐 Python | 📅 2023-07-04 ![](https://img.shields.io/github/stars/uhlik/bpy?color=yellow\&style=flat-square\&logo=github) - Point cloud importer/renderer/editor for Blender, Point Cloud visualizer.
* [3D Bounding Box Annotation Tool ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/walzimmer/3d-bat) ⭐ 798 | 🐛 45 | 🌐 TypeScript | 📅 2024-03-07 ![](https://img.shields.io/github/stars/walzimmer/3d-bat?color=yellow\&style=flat-square\&logo=github) - 3D BAT: A Semi-Automatic, Web-based 3D Annotation Toolbox for Full-Surround, Multi-Modal Data Streams, point cloud annotator / labeling.
  * [Paper ![](https://img.shields.io/badge/paper-blue?style=flat-square\&logo=semanticscholar)](https://arxiv.org/pdf/1905.00525.pdf)
  * [YouTube video ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/watch?v=gSGG4Lw8BSU)
* [Lichtblick suite](https://github.com/lichtblick-suite) - Lichtblick is an open-source alternative to Foxglove Studio for visualizing and analyzing robotics data.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/lichtblick-suite/lichtblick) ⭐ 749 | 🐛 28 | 🌐 TypeScript | 📅 2026-02-19 ![](https://img.shields.io/github/stars/lichtblick-suite/lichtblick?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
* [Pointcloudprinter ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/marian42/pointcloudprinter) ⭐ 163 | 🐛 0 | 🌐 C# | 📅 2021-01-17 ![](https://img.shields.io/github/stars/marian42/pointcloudprinter?color=yellow\&style=flat-square\&logo=github) - A tool to turn point cloud data from aerial lidar scans into solid meshes for 3D printing.
* [ARHeadsetKit](https://github.com/philipturner/ARHeadsetKit) ⚠️ Archived - Using $5 Google Cardboard to replicate Microsoft Hololens. Hosts the source code for research on [scene color reconstruction](https://github.com/philipturner/scene-color-reconstruction) ⭐ 25 | 🐛 0 | 📅 2021-10-26.
* [CloudPeek](https://github.com/Geekgineer/CloudPeek) ⭐ 126 | 🐛 0 | 🌐 C++ | 📅 2024-10-20 is a lightweight, c++ single-header, cross-platform point cloud viewer, designed for simplicity and efficiency without relying on heavy external libraries like PCL or Open3D.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/Geekgineer/CloudPeek) ⭐ 126 | 🐛 0 | 🌐 C++ | 📅 2024-10-20 ![](https://img.shields.io/github/stars/Geekgineer/CloudPeek?color=yellow\&style=flat-square\&logo=github)
* [Foxglove](https://foxglove.dev/) - Foxglove Studio is an integrated visualization and diagnosis tool for robotics, available in your browser or for download as a desktop app on Linux, Windows, and macOS.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/foxglove/studio) ⚠️ Archived ![](https://img.shields.io/github/stars/foxglove/studio?color=yellow\&style=flat-square\&logo=github) ![](https://img.shields.io/badge/ROS-2-34aec5?style=flat-square\&logo=ros)
  * [YouTube channel ![](https://img.shields.io/badge/youtube-red?style=flat-square\&logo=youtube)](https://www.youtube.com/channel/UCrIbrBxb9HBAnlhbx2QycsA)
* [CloudCompare](https://cloudcompare.org/) - CloudCompare is a free, cross-platform point cloud editor software.
  * [GitHub repository ![](https://img.shields.io/badge/github-black?style=flat-square\&logo=github)](https://github.com/CloudCompare) ![](https://img.shields.io/github/stars/CloudCompare?color=yellow\&style=flat-square\&logo=github)
* [Which SLAM Algorithm Should I Choose?](https://www.slambotics.org/blog/which-slam-to-choose) Slambotics -  Choosing the Right SLAM Algorithm

<img src="https://raw.githubusercontent.com/szenergy/awesome-lidar/refs/heads/main/img/cc0.svg" width="400" alt="CC0" />
