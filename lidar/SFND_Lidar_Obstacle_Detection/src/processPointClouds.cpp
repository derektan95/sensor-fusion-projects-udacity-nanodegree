// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// --- USING OWN IMPLEMENTATION --- //

// Own implementation of Plane Segmentation.
// Locates and separates plane from overall point cloud.
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneOwnImplementation(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //  Fill in this function to find inliers for the cloud - own implementation
    std::unordered_set<int> inliers = RansacOwnImplementation3D(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    if (cloudInliers->points.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

// Own implementation of Ransac 3D.
// Uses Ransac in 3D to identify plane.
template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacOwnImplementation3D(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations
    for (size_t i = 0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line (Add chosen ints into tempInliersResult)
        std::unordered_set<int> tempInliersResult;
        while (tempInliersResult.size() < 3)
        {
            tempInliersResult.insert((rand() % cloud->points.size()));
        } // avoid choosing duplicate element

        auto itr = tempInliersResult.begin();
        PointT randPoint1 = cloud->points[*itr];
        itr++;
        PointT randPoint2 = cloud->points[*itr];
        itr++;
        PointT randPoint3 = cloud->points[*itr];

        // Measure distance between every point and fitted line
        double planeAValue = ((randPoint2.y - randPoint1.y) * (randPoint3.z - randPoint1.z)) - ((randPoint2.z - randPoint1.z) * (randPoint3.y - randPoint1.y));
        double planeBValue = ((randPoint2.z - randPoint1.z) * (randPoint3.x - randPoint1.x)) - ((randPoint2.x - randPoint1.x) * (randPoint3.z - randPoint1.z));
        double planeCValue = ((randPoint2.x - randPoint1.x) * (randPoint3.y - randPoint1.y)) - ((randPoint2.y - randPoint1.y) * (randPoint3.x - randPoint1.x));
        double planeDValue = -((planeAValue * randPoint1.x) + (planeBValue * randPoint1.y) + (planeCValue * randPoint1.z));

        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            // Do not carry on this iteration if already in tempInliersResult
            if (tempInliersResult.count(i) > 0)
                continue;

            PointT point = cloud->points[i];
            double distance = fabs((planeAValue * point.x) + (planeBValue * point.y) + (planeCValue * point.z) + planeDValue) / (sqrt((planeAValue * planeAValue) + (planeBValue * planeBValue) + (planeCValue * planeCValue)));

            // If distance is smaller than threshold count it as inlier
            // NOTE: Would not insert if same int being inserted
            if (distance <= distanceTol)
                tempInliersResult.insert(i);
        }

        // Return indicies of inliers from fitted line with most inliers
        if (tempInliersResult.size() > inliersResult.size())
            inliersResult = tempInliersResult;
    }

    return inliersResult;
}

// Own implementation of Clustering 3D.
// Cluster point clouds representing obstacles.
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringOwnImplementation(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Instantiate 3d kd tree and initialize it from points in point cloud
    KdTree *tree3D = new KdTree;
    std::vector<std::vector<float>> pointsVec(cloud->points.size());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> pointVec = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree3D->insert(pointVec, i);
        pointsVec[i] = pointVec;
    }

    // Configure EuclideanCluster parameters
    std::vector<std::vector<int>> clusterIndices = EuclideanClusterOwnImplementation(pointsVec, tree3D, clusterTolerance, minSize, maxSize);

    // Iterate through each cluster in clusterIndices
    for (std::vector<int> cluster : clusterIndices)
    {
        // Populate cloudClusters with points in each cluster indice
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index : cluster)
        {
            cloudCluster->points.push_back(cloud->points[index]); //*
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        // Store point cloud cluster in vector of clusters
        clusters.push_back(cloudCluster);

        std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size() << " data points." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Own implementation of Euclidean Clustering 3D.
// Returns vector of cluster of points that are grouped together using euclideanCluster method.
// Makes sure that every point's neighbours are considered when clustering.
template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanClusterOwnImplementation(const std::vector<std::vector<float>> &points, KdTree *&tree, float distanceTol, float minSize, float maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> isProcessedVector(points.size(), false);

    for (size_t i = 0; i < points.size(); i++)
    {
        if (!isProcessedVector[i])
        {
            std::vector<int> cluster;
            UpdateCluster(points, tree, distanceTol, i, cluster, isProcessedVector);
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
                clusters.push_back(cluster);
        }
    }

    return clusters;
}

// Recursive method called by euclideanCluster function.
// Initializes cluster vector passed in. Also updates isProcessedVector to keep track of processed points.
template <typename PointT>
void ProcessPointClouds<PointT>::UpdateCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int pointIdx, std::vector<int> &cluster, std::vector<bool> &isProcessedVector)
{
    if (!isProcessedVector[pointIdx])
    {
        isProcessedVector[pointIdx] = true;
        cluster.push_back(pointIdx);
        std::vector<int> potentialPointClusterIdx = tree->search(points[pointIdx], distanceTol);

        for (int idx : potentialPointClusterIdx)
        {
            if (!isProcessedVector[idx])
            {
                UpdateCluster(points, tree, distanceTol, idx, cluster, isProcessedVector);
            }
        }
    }
}

// --- USING INSTRUCTOR'S IMPLEMENTATION --- //

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());

    // Filter based on box parameters - exclude everything outside of box to minimize num points
    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*filteredCloud);

    // Filter using voxel grid method - make points more sparse
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(filteredCloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filteredCloud);

    // Remove points from car which lidar is mounted on - inverse box filter method
    std::vector<int> indices;
    pcl::CropBox<PointT> roofBoxFilter(true);
    roofBoxFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofBoxFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofBoxFilter.setInputCloud(filteredCloud);
    roofBoxFilter.filter(indices); // Store index of points to be filtered

    // Define inliers for separation (like in plane segmentation)
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int pointIdx : indices)
        inliers->indices.push_back(pointIdx);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // typename pcl::PointCloud<PointT>::Ptr obstaclesPointCloud;       // NOTE: THIS IS WRONG, POINTER MUST POINT TO SOMETHING!

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    // Deep copy points from inliers to obstacle point cloud
    for (size_t i = 0; i < inliers->indices.size(); i++)
    {
        PointT point = cloud->points[inliers->indices[i]];
        planeCloud->points.push_back(point);
    }
    // for (int index : inliers->indices) planeCloud->points.push_back(cloud->points[index]);       // THIS METHOD IS FASTER

    // Create the filtering object & Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // to contain segmented plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // contains information about the plane
    pcl::SACSegmentation<PointT> seg;

    // Configure parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // Random Sample Concensus
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Configure EuclideanCluster parameters
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // Iterate through each cluster in clusterIndices
    for (pcl::PointIndices getIndices : clusterIndices)
    {
        // Populate cloudClusters with points in each cluster indice
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]); //*
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        // Store point cloud cluster in vector of clusters
        clusters.push_back(cloudCluster);

        std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size() << " data points." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
