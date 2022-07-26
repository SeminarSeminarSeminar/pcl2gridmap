#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "header/heatmap_filtering.hpp"

#define HEAT_MAP_FILTER 1
int main (int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	std::string pcl_path ="/home/alpha/Desktop/DB/MapPoints.txt";
	std::string mean_k_val = "10";
	std::string thresh = "1.0";

	reader.read<pcl::PointXYZ> (pcl_path, *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (std::stoi(mean_k_val));
	sor.setStddevMulThresh (std::stof(thresh));
	sor.filter (*cloud_filtered);

	#if HEAT_MAP_FILTER /* heat-map filtering */
	decode_point_cloud(cloud_filtered, 5);
	#endif 

	
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud_filtered);
	while (!viewer.wasStopped ())
	{
	}

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("map_inliners.pcd", *cloud_filtered, false);
	std::cout<<"map writing complete"<<std::endl;
	return (0);
}
