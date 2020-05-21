# lidar-obstacle-detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Obstacle detection using lidar data

Processing point clouds, and use it to detect car and trucks on a narrow street using lidar.

Detection pipeline : filtering -> segmentation -> clustering -> bounding boxes.

Segmentation and clustering methods are created from scratch.

### Segmentation

Point clouds is separated into obstable cloud and plane cloud using RANSAC algorithm implemented
from scratch.

Below is the simulated scenario shown with lidar rays rendered

<img src="media/scenarioWithLidarRays.png" width="700" height="400" />

Following is the segmentation output, as seen in the image point clouds is segmented into
two parts as plane cloud and obstacle cloud. All the points that belong to plane cloud
(Road surface) is shown in green and the obstacle cloud is rendered in red color.

<img src="media/segmentation.png" width="700" height="400" />

### Clustering

Euclidean Clustering method implemented using KD tree is used to cluster the obstable cloud
obtained from Segmentation. Which is later used to track objects. 

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