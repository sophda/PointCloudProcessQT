//
// Created by sophda on 4/24/23.
//

#ifndef QT_NODEDATA_H
#define QT_NODEDATA_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>
#include "Macro.h"
void create();

class DataNode
{
public:

    DataNode *node_child=nullptr,*node_parent=nullptr;
    cv::Mat image_origin;
    cv::Mat image_gray;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = nullptr;  // using the template pcl::pointxyz in the class pointcloud ,then the shared ptr in this namespace will be used
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = nullptr;
    int fun_name,class_name;

    explicit DataNode();
    explicit DataNode(int fun_input,int class_input)
    :fun_name{fun_input},class_name{class_input}
    {};
};


#endif //QT_NODEDATA_H
