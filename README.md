# lidar-obstacle-detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Obstacle detection using lidar data (Part of Udacity's Sensor Fusion Nano Degree)

Processing point clouds, and use it to detect car and trucks on a narrow street using lidar.

Detection pipeline : filtering -> segmentation -> clustering -> bounding boxes.

Segmentation and clustering methods are created from scratch.

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
```

### MAC (via Homebrew)

```bash
$> brew tap brewsci/science
$> brew install pcl
```


## Build and Run

```bash
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```