/* \author Ragu Manjegowda */
/* \adopted from Aaron Brown */
// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region
    // based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
    // Done: Create two new point clouds, one cloud with obstacles and other
    // with segmented plane
    auto obstacleCloud = new pcl::PointCloud<PointT>();
    auto planeCloud = new pcl::PointCloud<PointT>();

    for (auto it : inliers->indices) {
        planeCloud->points.push_back(cloud->points[it]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
        segResult(obstacleCloud, planeCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                             int maxIterations,
        float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Done:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    /*pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset."
                  << std::endl;
    }*/

    // Implement RANSAC

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    int randomSampleIndex[3];

    // For max iterations
    while (maxIterations-- > 0) {
        // Randomly sample subset and fit plane
        pcl::PointIndices::Ptr inliersIntermediate(new pcl::PointIndices());
        int index = 0;
        while (inliersIntermediate->indices.size() < 3) {
            randomSampleIndex[index] = rand() % cloud->points.size();
            inliersIntermediate->indices.push_back(randomSampleIndex[index]);
            ++index;
        }

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        index = 0;
        y1 = cloud->points[randomSampleIndex[index]].y;
        z1 = cloud->points[randomSampleIndex[index]].z;
        x1 = cloud->points[randomSampleIndex[index]].x;
        ++index;
        x2 = cloud->points[randomSampleIndex[index]].x;
        y2 = cloud->points[randomSampleIndex[index]].y;
        z2 = cloud->points[randomSampleIndex[index]].z;
        ++index;
        x3 = cloud->points[randomSampleIndex[index]].x;
        y3 = cloud->points[randomSampleIndex[index]].y;
        z3 = cloud->points[randomSampleIndex[index]].z;

        float a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
        float b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
        float c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
        float d = -((a * x1) + (b * y1) + (c * z1));

        /*
         * For all x, y, z in point cloud check if it under tolerance
         * ∣A*x+B*y+C*z+D∣/sqrt(A^2+B^2+C^2)
         */
        for (index = 0; index < cloud->points.size(); ++index) {
            // If the point is one already picked as inlier skip it
            if (index == randomSampleIndex[0] || index == randomSampleIndex[1] ||
                index == randomSampleIndex[2]) {
                continue;
            }

            auto point = cloud->points[index];

            // Measure distance between every point and fitted line
            float distance = fabs((a * point.x) + (b * point.y) + (c * point.z) + d) /
                sqrt(a * a + b * b + c * c);

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceThreshold) {
                inliersIntermediate->indices.push_back(index);
            }

            // Set indicies of inliers from fitted line with most inliers
            if (inliersIntermediate->indices.size() > inliers->indices.size()) {
                inliers = inliersIntermediate;
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds"
              << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
        segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance,
    int minSize,
    int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group
    // detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found "
              << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file
              << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
              << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {

    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{dataPath},
        boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}