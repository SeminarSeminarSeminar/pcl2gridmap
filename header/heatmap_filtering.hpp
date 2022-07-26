#ifndef HEATMAP_HPP
#define HEATMAP_HPP

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
void decode_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, int thres);


#endif