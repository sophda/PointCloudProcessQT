//
// Created by sophda on 2/28/23.
//

#ifndef QT_POINTCLOUD_H
#define QT_POINTCLOUD_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <QFileInfo>
#include <pcl/common/common.h>
using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class pointCloud
{
public:

    pointCloud();
    ~pointCloud();


//    PointCloudT::Ptr cloud = PointCloudT::Ptr(new PointCloudT);

    PointCloudT::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    pcl::PolygonMesh::Ptr mesh;
    long double area, volume;


    float calculateVolumeInt(pcl::PointCloud<pcl::PointXYZ>::Ptr &input);
    void passThrough(PointCloudT::Ptr &output);
    void staticFilter(PointCloudT::Ptr &output);
//    void twosideFilter(PointCloudT::Ptr input, PointCloudT::Ptr output);
    void calculateArea();
    void pcl2Mesh();
    void initPoint(string pcdname);
    void calculateVolume();

    void oushiFilter(std::vector<PointCloudT::Ptr>& pointcloudVec);

};
#endif //QT_POINTCLOUD_H
