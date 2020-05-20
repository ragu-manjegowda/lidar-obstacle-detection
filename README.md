# lidar-obstacle-detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Obstacle detection using lidar data (Part of Udacity's Sensor Fusion Nano Degree)

Processing point clouds, and use it to detect car and trucks on a narrow street using lidar.

Detection pipeline : filtering -> segmentation -> clustering -> bounding boxes.

Segmentation and clustering methods are created from scratch.

## Installation

### Docker

```bash
$> docker pull ragumanjegowda/docker:latest
```

### Ubuntu 

```bash
$> sudo apt install libpcl-dev ninja-build cmake
```

### MAC (via Homebrew)

```bash
$> brew tap brewsci/science
$> brew install pcl
$> brew install ninja cmake
```


## Build and Run

```bash
$> mkdir build && cd build
$> cmake -G Ninja ..
$> ninja -j400
$> ./environment
```