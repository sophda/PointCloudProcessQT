//
// Created by sophda on 4/25/23.
//

#ifndef QT_FUNTIONCLOUD_H
#define QT_FUNTIONCLOUD_H

#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/passthrough.h>
#include <memory>
#include <pcl/features/rops_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/search/flann_search.h>
#include <pcl/filters/bilateral.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
//#include <atlstr.h>//CString头文件
#include <vtkPLYReader.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>
#include <vtkMassProperties.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "DBSCAN.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
void statistic(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
void cloud2mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PolygonMesh &mesh);
void calArea(pcl::PolygonMesh &mesh, float *area);
void calVolume(pcl::PolygonMesh &mesh,float *area);
void fangzheng(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
void touying(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,const cv::Mat& out);
void shengzhangquyu(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                    std::vector <pcl::PointIndices> & clusters);
void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
               std::vector<float> angle,Eigen::Matrix3f matrix,
               pcl::PointCloud<pcl::PointXYZ>::Ptr & output);
Eigen::Matrix3f getTransformMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
               std::vector<float> & angle);

void randomTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
void dbscanFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud_input,
                  std::vector<pcl::PointCloud<pcl::PointXYZ> > & output);
void getHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_input,
               float *height);
void getOneusingDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud_input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr & output);
void ply2pcd();
void shengzhang2();
void oushi();
#endif //QT_FUNTIONCLOUD_H
