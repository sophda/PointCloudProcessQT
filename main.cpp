#include <QCoreApplication>
#include <iostream>
#include <QDebug>
#include <QApplication>
#include "mainwindow.h"
#include "src/pointCloud.h"
#include "src/NodeData.h"
#include "src/FuntionCloud.h"
#include "src/DimWindow.h"
#include <iostream>
#include <fstream>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


using namespace  std;
void test();
void test1();
void txt2pcd();
void testTransform();
int main(int argc, char *argv[]) {

//    ply2pcd();
//    txt2pcd();
//    test();
//    pcl::PCDtoPLYconvertor("../file/single.pcd","../file/single.ply")
//    shengzhang2();
//    oushi();
    test1();
//    testTransform();
    if (0)
    {
        QApplication a(argc, argv);
        mainwindow w;
//        DimWindow w1;
//        w1.show();
        w.show();
        return QApplication::exec();
    }
}


void test()
{
    //    createLink();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../file/points.pcd",*cloud);

//    ply2pcd();

//    randomTransform(cloud,cloud_);
    fangzheng(cloud,cloud_);
    passthrough(cloud_,cloud_);
//    statistic(cloud_,cloud_);
//    statistic(cloud_,cloud_);
//    statistic(cloud_,cloud_);
    std::vector<pcl::PointCloud<pcl::PointXYZ> >  output;
    dbscanFilter(cloud_,output);

//    statistic(cloud,cloud_);
//    statistic(cloud_,cloud_);
//    pcl::io::savePCDFileASCII("../file/passsta.pcd", *cloud_);

    for (auto i = 0;i<output.size();i++)
    {
        if (i==0)
            *cloud=output.at(0);
        else
        {
            *cloud+=output.at(i);
        }
    }
    fangzheng(cloud,cloud);


//    std::cout<< "output : " << output.size() << std::endl;
//    *cloud = output.at(0);
//    *cloud += output.at(1);
//    *cloud += output.at(3);

//    *cloud += output.at(4);

    pcl::io::savePCDFileASCII("../file/single.pcd",*cloud);

    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Ransac"));
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            single_color_handler_(cloud_, 255, 0, 0);
    viewer->addPointCloud(cloud_,single_color_handler_, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            single_color_handler(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud,single_color_handler,"fff");
    viewer->spin();


}

void txt2pcd()
{
    std::ifstream input("../file/points3D.txt");
    if (!input.good())
    {
        std::cerr << "Could not read: "  << std::endl;
//        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 0;
    cloud.height = 1;
    int id;
    std::string line;
    while (std::getline(input, line))
    {
        pcl::PointXYZ point;
        std::istringstream iss(line);
        iss>>id >> point.x >> point.y >> point.z ;
        cloud.push_back(point);
        ++cloud.width;
    }
    cloud.width--;
    cout<<cloud.width<<endl;
    pcl::io::savePCDFileASCII("../file/points.pcd", cloud);
}

void test1()
{
    //    createLink();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../file/single.pcd",*cloud);
    pcl::PointXYZ minPt, maxPt;
    // pcl::getMinMax3D (*cloud, minPt, maxPt);
    statistic(cloud,cloud);
    statistic(cloud,cloud);
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    printf("%f",-minPt.x+maxPt.x);
//    ply2pcd();
//    randomTransform(cloud,cloud_);
//    fangzheng(cloud,cloud_);
//    passthrough(cloud,cloud_);
//    statistic(cloud,cloud_);
    pointCloud class_pcd;
    class_pcd.initPoint("../file/single.pcd");
    class_pcd.pcl2Mesh();
//    pcl::io::savePCDFileASCII("../file/tongji.pcd",*cloud_);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));    viewer->setRepresentationToWireframeForAllActors();
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(255, 255, 255);
//    viewer->addPointCloud(cloud, "cloud");

    viewer->addPolygonMesh(*class_pcd.mesh);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//            single_color_handler(cloud_, 0, 255, 0);
//    viewer->addPointCloud(cloud_,single_color_handler,"fff");
    viewer->spin();


}

void testTransform()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../file/girl.pcd",*cloud);
    randomTransform(cloud,cloud_);

    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Ransac"));
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            single_color_handler(cloud_, 0, 255, 0);
    viewer->addPointCloud(cloud_,single_color_handler,"fff");
    viewer->spin();

}