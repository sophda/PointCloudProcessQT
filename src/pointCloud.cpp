//
// Created by sophda on 2/28/23.
//

#include "pointCloud.h"
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


using namespace std;
float heronArea(pcl::PolygonMesh mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *cloud);

    std::vector<pcl::Vertices>::iterator it;
    int i;
    long double sums = 0;
    for (it = mesh.polygons.begin(); i < mesh.polygons.size(); i++)
    {

        long double ax, ay, az, bx, by, bz, cx, cy, cz;
        //ax = it[i].vertices[0];

        ax = cloud->points[it[i].vertices[0]].x;
        ay = cloud->points[it[i].vertices[0]].y;
        az = cloud->points[it[i].vertices[0]].z;
        bx = cloud->points[it[i].vertices[1]].x;
        by = cloud->points[it[i].vertices[1]].y;
        bz = cloud->points[it[i].vertices[1]].z;
        cx = cloud->points[it[i].vertices[2]].x;
        cy = cloud->points[it[i].vertices[2]].y;
        cz = cloud->points[it[i].vertices[2]].z;


        long double a = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
        long double b = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy) + (az - cz) * (az - cz));
        long double c = sqrt((cx - bx) * (cx - bx) + (cy - by) * (cy - by) + (cz - bz) * (cz - bz));

        long double p = (a + b + c) / 2;

        long double s = sqrt(p * (p - a) * (p - b) * (p - c));

        sums = sums + s;

    }
    return sums;
}
pcl::PolygonMesh getMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)//, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{

    std::cout << cloud_in->points.size() << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_in);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setKSearch(10);
    //normalEstimation.setRadiusSearch(0.03);
    normalEstimation.compute(*normals);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

    //定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.1);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);

    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);

    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    cloud_with_normals->width = cloud_with_normals->height = 0;
//    std::cout << "success traingles, time(s) "<< time.getTimeSeconds() << std::endl;

//    pcl::io::savePLYFile("../tmp.ply", triangles);
    return triangles;
}
pointCloud::pointCloud() {
//    PointCloudT::Ptr cloudtmp (new PointCloudT);
//    cloud = cloudtmp;


    cloud.reset(new PointCloudT);
    mesh.reset(new pcl::PolygonMesh);
}
pointCloud::~pointCloud() {

}
void pointCloud::calculateArea() {

   // pcl::io::loadPolygonFile(infile, mesh);
    pcl2Mesh();
    cout << "Polygon: " << mesh->polygons.size() << std::endl;
    cout << "cloud: " << mesh->cloud.width << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh->cloud, *cloudxyz);

    vector<pcl::Vertices>::iterator it;
    int i=0;
    long double sums = 0;
    cout<<"daocci"<<endl;
    long double ax, ay, az,bx,by,bz,cx,cy,cz = 0;
    cout<<mesh->polygons.size()<<endl;
    for (it = mesh->polygons.begin();i<mesh->polygons.size();i++)
    {


        //ax = it[i].vertices[0];
//        cout<<i<<endl;
        ax = cloudxyz->points[it[i].vertices[0]].x;
        ay = cloudxyz->points[it[i].vertices[0]].y;
        az = cloudxyz->points[it[i].vertices[0]].z;
        bx = cloudxyz->points[it[i].vertices[1]].x;
        by = cloudxyz->points[it[i].vertices[1]].y;
        bz = cloudxyz->points[it[i].vertices[1]].z;
        cx = cloudxyz->points[it[i].vertices[2]].x;
        cy = cloudxyz->points[it[i].vertices[2]].y;
        cz = cloudxyz->points[it[i].vertices[2]].z;


        long double a = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
        long double b = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy) + (az - cz) * (az - cz));
        long double c = sqrt((cx - bx) * (cx - bx) + (cy - by) * (cy - by) + (cz - bz) * (cz - bz));

        long double p = (a+b+c)/2;

        long double s = sqrt(p * (p - a) * (p - b) * (p - c));

        sums = sums + s;

    }
//    cout << " 面积为 "<< sums << " ";
    area = sums;

}



void pointCloud::pcl2Mesh() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
    int M = cloud->points.size();
    cout<<"input size is:"<<M<<endl;

    for (int i = 0;i <M;i++)
    {
        pcl::PointXYZ p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        output->points.push_back(p);
    }
    output->width = 1;
    output->height = M;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_1->setInputCloud(output);
    n.setInputCloud(output);
    n.setSearchMethod(tree_1);
    n.setKSearch(20);
    n.compute(*normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*output, *normals, *cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(200.0f);
    gp3.setMu(2.5f);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*mesh);
}

void pointCloud::initPoint(string pcdname) {
    if (pcl::io::loadPCDFile(pcdname, *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file!");
        system("pause");
        std::cout<<"error"<<std::endl;
    }
}

void pointCloud::calculateVolume() {

    cout<< cloud->points.size() <<endl;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCDReader reader;
//    reader.read<pcl::PointXYZ>("E:\\vs_code\\hello_pcl\\hello_pcl\\table_scene_lms400.pcd",*cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(1.0);
    coefficients->values.push_back(0.0);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setInputCloud(cloud_xyz);
    proj.setModelCoefficients(coefficients);
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.filter(*cloud_projected);



}

float pointCloud::calculateVolumeInt(pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
{
     float length=1; char order='z';
    //  input cloud is transformed cloud
    pcl::PointXYZ min_pt,max_pt;
    float max,min;
    pcl::getMinMax3D(*input,min_pt,max_pt);
    std::cout<<min_pt.y<<"   "<<max_pt.y;

    max=max_pt.y;
    min=min_pt.y;
//    switch (order)
//    {
//        case 'z':
//            max=max_pt.z;
//            min=min_pt.z;
//        case 'y':
//            max=max_pt.y;
//            min=min_pt.y;
//        case 'x':
//            max=max_pt.x;
//            min=min_pt.x;
//        default:
//            std::cout <<"Error with order"<< endl;
//    }

    int point_size = input->points.size();
    float iterator = min;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tmp_cloud->width = input->width;
    tmp_cloud->height = input->height;
    tmp_cloud->points.resize(tmp_cloud->width*tmp_cloud->height);
    tmp_cloud->is_dense = false;
    float sum_area,sum_volume;
    while (iterator<max)
    {
        std::cout<<iterator<<endl;
        for (int i=0;i<point_size;i++)
        {
            // this loop will copy the useful point cloud into tmp_cloud which will be calculated,
            // and the depth(x,y,z) will be ignored so that it can be regarded as a plane
            if (input->points[i].y<iterator+length && input->points[i].y>iterator)
            {
                tmp_cloud->points[i].y = iterator;
                tmp_cloud->points[i].x = input->points[i].x;
                tmp_cloud->points[i].z = input->points[i].z;
            }

        }
        pcl::PolygonMesh tmp_mesh;
        if (tmp_cloud->points.size()<5)
        {continue;}
        else{
            tmp_mesh = getMesh(tmp_cloud);
            float area = heronArea(tmp_mesh);
            sum_volume += area*length;
        }

        tmp_cloud.reset();
        iterator += length;
    }

    return sum_volume;
}
void pointCloud::passThrough( PointCloudT::Ptr &output) {


    // create pointcloud ptr
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);

    pass.setFilterFieldName("z");
//    pass.setFilterLimits()
    pass.filter(*output);

}

void pointCloud::staticFilter(PointCloudT::Ptr &output) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*output);

}

void pointCloud::oushiFilter(std::vector<PointCloudT::Ptr>& pointcloudVec) {




}


