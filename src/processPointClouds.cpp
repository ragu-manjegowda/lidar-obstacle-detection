/* \author Ragu Manjegowda */
/* \adopted from Aaron Brown */
// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "kdtree.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds()
{
}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds()
{
}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Done:: Fill in the function to do voxel grid point reduction and region
    // based filtering

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    /*
     * Voxel filtering. Reference
     * https://pcl-tutorials.readthedocs.io/en/latest/voxel_grid.html?highlight=voxel
     */
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    /*
     * CropBox. Reference
     * https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
     */
    typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_filtered);
    roi.filter(*cloud_roi);

    // Remove points hitting roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_roi);
    roof.filter(indices);

    // Create the filtering object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto point : indices)
    {
        inliers->indices.push_back(point);
    }

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roi);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Done: Create two new point clouds, one cloud with obstacles and other
    // with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);

    for (auto it : inliers->indices)
    {
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
                                             float distanceThreshold)
{
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
    while (maxIterations-- > 0)
    {
        // Randomly sample subset and fit plane
        pcl::PointIndices::Ptr inliersIntermediate(new pcl::PointIndices());
        int index = 0;
        while (inliersIntermediate->indices.size() < 3)
        {
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
        for (index = 0; index < cloud->points.size(); ++index)
        {
            // If the point is one already picked as inlier skip it
            if (index == randomSampleIndex[0] || index == randomSampleIndex[1] ||
                index == randomSampleIndex[2])
            {
                continue;
            }

            auto point = cloud->points[index];

            // Measure distance between every point and fitted line
            float distance = fabs((a * point.x) + (b * point.y) + (c * point.z) + d) /
                             sqrt(a * a + b * b + c * c);

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceThreshold)
            {
                inliersIntermediate->indices.push_back(index);
            }

            // Set indicies of inliers from fitted line with most inliers
            if (inliersIntermediate->indices.size() > inliers->indices.size())
            {
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
void euclideanClusterHelper(
    int index,
    std::vector<int>& cluster,
    std::vector<bool>& visited,
    const std::vector<PointT, Eigen::aligned_allocator<PointT>>& points,
    KdTree<PointT>* tree,
    float distanceTol)
{
    if (visited[index] == true)
    {
        return;
    }

    visited[index] = true;
    cluster.push_back(index);

    std::vector<int> nearestPoints = tree->search(points.at(index), distanceTol);

    for (auto& it : nearestPoints)
    {
        if (!visited[it])
        {
            euclideanClusterHelper(it, cluster, visited, points, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>> euclideanCluster(
    const std::vector<PointT, Eigen::aligned_allocator<PointT>>& points,
    KdTree<PointT>* tree,
    float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> visited(points.size(), false);

    for (int i = 0; i < points.size(); i++)
    {
        if (visited[i] == false)
        {
            std::vector<int> cluster;
            euclideanClusterHelper(i, cluster, visited, points, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance,
    int minSize,
    int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Done:: Fill in the function to perform euclidean clustering to group
    // detected obstacles
    // Creating the KdTree object for the search method of the extraction

    /*typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);  // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }*/

    // Implement Euclidean clustering with KD-Tree
    KdTree<PointT>* tree = new KdTree<PointT>();

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        tree->insert(cloud->points[i], i);
    }

    std::vector<std::vector<int>> clusterIndices =
        euclideanCluster(cloud->points, tree, clusterTolerance);

    for (auto cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice : cluster)
        {
            clusterCloud->points.push_back(PointT(cloud->points.at(indice)));
        }
        if (clusterCloud->points.size() >= minSize &&
            clusterCloud->points.size() <= maxSize)
        {
            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            clusters.push_back(clusterCloud);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found "
              << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(
    typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Project the cluster onto the XY plane
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr clusterXYProjection(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (PointT point : cluster->points)
    {
        clusterXYProjection->points.push_back(pcl::PointXYZ(point.x, point.y, 0));
    }

    /*
     * Reference -
     * http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
     */
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clusterXYProjection, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clusterXYProjection, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) =
        -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(
        new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    const Eigen::Vector3f meanDiagonal =
        0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    BoxQ boxQ;

    boxQ.bboxQuaternion = eigenVectorsPCA;
    boxQ.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    boxQ.cube_length = maxPoint.x - minPoint.x;
    boxQ.cube_width = maxPoint.y - minPoint.y;
    boxQ.cube_height = maxPoint.z - minPoint.z;

    return boxQ;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file
              << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file)
{

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
    std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{dataPath},
        boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}