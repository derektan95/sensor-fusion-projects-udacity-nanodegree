// ORIGINAL NAME: RANSAC 2D
/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (size_t i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line (Add chosen ints into tempInliersResult)
		std::unordered_set<int> tempInliersResult;
		while (tempInliersResult.size() < 2) { tempInliersResult.insert((rand() % cloud->points.size())); }	// avoid choosing duplicate element	
		
		auto itr = tempInliersResult.begin();
		pcl::PointXYZ randomPoint1 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ randomPoint2 = cloud->points[*itr];

		// Measure distance between every point and fitted line
		double lineAValue = randomPoint1.y - randomPoint2.y;
		double lineBValue = randomPoint2.x - randomPoint1.x;
		double lineCValue = (randomPoint1.x * randomPoint2.y) - (randomPoint2.x * randomPoint1.y);
		
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			// Do not carry on this iteration if already in tempInliersResult
			if (tempInliersResult.count(i) > 0) continue;
			
			pcl::PointXYZ point = cloud->points[i];
			double distance = fabs((lineAValue * point.x) + (lineBValue * point.y) + (lineCValue)) / (sqrt((lineAValue*lineAValue) + (lineBValue*lineBValue)));

			// If distance is smaller than threshold count it as inlier
			// NOTE: Would not insert if same int being inserted
			if (distance <= distanceTol) tempInliersResult.insert(i);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (tempInliersResult.size() > inliersResult.size()) inliersResult = tempInliersResult;
	}

	return inliersResult;
}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		pcl::PointXYZ randPoint1 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ randPoint2 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ randPoint3 = cloud->points[*itr];

		// Measure distance between every point and fitted line
		double planeAValue = ((randPoint2.y-randPoint1.y)*(randPoint3.z-randPoint1.z)) - ((randPoint2.z-randPoint1.z)*(randPoint3.y-randPoint1.y));
		double planeBValue = ((randPoint2.z-randPoint1.z)*(randPoint3.x-randPoint1.x)) - ((randPoint2.x-randPoint1.x)*(randPoint3.z-randPoint1.z));
		double planeCValue = ((randPoint2.x-randPoint1.x)*(randPoint3.y-randPoint1.y)) - ((randPoint2.y-randPoint1.y)*(randPoint3.x-randPoint1.x));
		double planeDValue = -((planeAValue*randPoint1.x) + (planeBValue*randPoint1.y) + (planeCValue*randPoint1.z));

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			// Do not carry on this iteration if already in tempInliersResult
			if (tempInliersResult.count(i) > 0) continue;
			
			pcl::PointXYZ point = cloud->points[i];
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


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data - 2D / 3D
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function - 2D / 3D
	// std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

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



	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
