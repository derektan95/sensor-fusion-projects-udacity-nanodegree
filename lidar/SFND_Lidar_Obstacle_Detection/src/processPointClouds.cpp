// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


// --- USING PCL RANSAC IMPLEMENTATION --- //

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // typename pcl::PointCloud<PointT>::Ptr obstaclesPointCloud;       // NOTE: THIS IS WRONG, POINTER MUST POINT TO SOMETHING!

    typename pcl::PointCloud<PointT>::Ptr obstCloud  ( new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud ( new pcl::PointCloud<PointT> ());
    
    // Deep copy points from inliers to obstacle point cloud
    for (size_t i = 0; i < inliers->indices.size(); i++)
    {
        PointT point = cloud->points[inliers->indices[i]];
        planeCloud->points.push_back(point);
    }
    // for (int index : inliers->indices) planeCloud->points.push_back(cloud->points[index]);       // THIS METHOD IS FASTER

    // Create the filtering object & Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud );
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);     // to contain segmented plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);      // contains information about the plane
    pcl::SACSegmentation<PointT> seg;

    // Configure parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);            // Random Sample Concensus
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from input cloud
    seg.setInputCloud (cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        exit (-1);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


// --- USING OWN RANSAC IMPLEMENTATION --- //

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneOwnImplementation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //  Fill in this function to find inliers for the cloud - own implementation
    std::unordered_set<int> inliers = RansacOwnImplementation3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    if (cloudInliers->points.size() == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        exit (-1);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacOwnImplementation3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (size_t i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line (Add chosen ints into tempInliersResult)
		std::unordered_set<int> tempInliersResult;
		while (tempInliersResult.size() < 3) { tempInliersResult.insert((rand() % cloud->points.size())); }	// avoid choosing duplicate element	
		
		auto itr = tempInliersResult.begin();
		PointT randPoint1 = cloud->points[*itr];
		itr++;
		PointT randPoint2 = cloud->points[*itr];
		itr++;
		PointT randPoint3 = cloud->points[*itr];

		// Measure distance between every point and fitted line
		double planeAValue = ((randPoint2.y-randPoint1.y)*(randPoint3.z-randPoint1.z)) - ((randPoint2.z-randPoint1.z)*(randPoint3.y-randPoint1.y));
		double planeBValue = ((randPoint2.z-randPoint1.z)*(randPoint3.x-randPoint1.x)) - ((randPoint2.x-randPoint1.x)*(randPoint3.z-randPoint1.z));
		double planeCValue = ((randPoint2.x-randPoint1.x)*(randPoint3.y-randPoint1.y)) - ((randPoint2.y-randPoint1.y)*(randPoint3.x-randPoint1.x));
		double planeDValue = -((planeAValue*randPoint1.x) + (planeBValue*randPoint1.y) + (planeCValue*randPoint1.z));

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			// Do not carry on this iteration if already in tempInliersResult
			if (tempInliersResult.count(i) > 0) continue;
			
			PointT point = cloud->points[i];
			double distance = fabs((planeAValue * point.x) + (planeBValue * point.y) + (planeCValue * point.z) + planeDValue) / (sqrt((planeAValue*planeAValue) + (planeBValue*planeBValue) + (planeCValue*planeCValue)));

			// If distance is smaller than threshold count it as inlier
			// NOTE: Would not insert if same int being inserted
			if (distance <= distanceTol) tempInliersResult.insert(i);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (tempInliersResult.size() > inliersResult.size()) inliersResult = tempInliersResult;
	}

	return inliersResult;
}