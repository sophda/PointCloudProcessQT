//
// Created by sophda on 4/25/23.
//

#include "FuntionCloud.h"
using namespace std;
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    cout<<"zhitong lvbo:"<<input->points.size()<<endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(5.6, 10.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*output);

}
void statistic(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    cout<<"tongj i a"<<endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*output);
}
void shuangbianlvbo(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output){
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new  pcl::search::KdTree<pcl::PointXYZ>);
//    tree1->setInputCloud(input);
//    pcl::BilateralFilter<pcl::PointXYZ> nf;
//    nf.setInputCloud(input);
//    nf.setSearchMethod(tree1);
//    nf.setHalfSize(1);
//    nf.setStdDev(0.01);
//    nf.filter(*output);

}

void shengzhangquyu(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                    std::vector <pcl::PointIndices> & clusters)
{
    // 创建一个空的kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // 创建一个normal点云
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud <pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator; // 创建法线估计对象
    normal_estimator.setSearchMethod (tree); // 设置搜索方法
    normal_estimator.setInputCloud (input); // 设置法线估计对象输入点集
    normal_estimator.setRadiusSearch (0.02); // 使用半径在查询点周围2厘米范围内的所有邻元素
    normal_estimator.compute (*cloud_normals); // 计算并输出法向

    int cloud_size = input->points.size();
    int class_size = int(cloud_size/50);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg; // 创造区域生长分割对象
    reg.setMinClusterSize (class_size); // 设置一个聚类需要的最小点数，聚类小于阈值的结果将被舍弃
    reg.setMaxClusterSize (1000000); //设置一个聚类需要的最大点数，聚类大于阈值的结果将被舍弃
    reg.setSearchMethod (tree); // 设置搜索方法
    reg.setResidualThreshold (0.03); // 设置搜索的近邻点数目

    reg.setInputCloud (input); // 设置输入点云
    reg.setInputNormals (cloud_normals); // 设置输入点云

    reg.setSmoothnessThreshold (5 / 180.0 * M_PI); //设置平滑阀值
    reg.setCurvatureThreshold (1); //设置曲率阀值


    // 以下两行用于启动分割算法，并返回聚类向量
//    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters); // 获取聚类的结果，分割结果保存在点云索引的向量中


//    std::vector<pcl::PointIndices> cluster;
//    shengzhangquyu(cloud,cluster);

//    cout<<cluster.at(6)<<endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = clusters[0].indices;
    inliers->header = clusters[0].header;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云


   // you need to identify the cloud_ as the output of these indices
//    extract.filter(*cloud_);


}
void cloud2mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PolygonMesh &mesh)
{
    cout<<"yun zhuan mesh"<<endl;
}
void calArea(pcl::PolygonMesh &mesh, double *area, double *vol)
{
    cout<<"suanmianji "<<endl;
//    pcl::PolygonMesh mesh;
//    mesh=getMesh(input);

//    pcl::io::savePLYFile ("../tmp.ply", *input);
    std::string inputFilename = "/home/sophda/project/pclTest/tmp.obj";


    //vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
//    reader->SetFileName(inputFilename.c_str());
    reader->SetFileName("../tmp.ply");
    reader->Update();

    vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
    tri->SetInputData(reader->GetOutput());
    tri->Update();
    vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
    poly->SetInputData(tri->GetOutput());
    poly->Update();

    *vol = poly->GetVolume();
    *area = poly->GetSurfaceArea();

//    cout << "体积为：" << vol << endl;
//    cout << "表面积为：" << area << endl;
}
void calVolume(pcl::PolygonMesh &mesh,float *area)
{
    cout << "suantiji" <<endl;
}
void fangzheng(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    pcl::PointCloud<pcl::PointXYZ>::PointType cloud;
    cout<<"fangzheng "<<endl;
    std::vector<float> mat;
    Eigen::Matrix3f matrix = getTransformMatrix(input, mat);
//    mat={1.6,0.6,0.5,
//         5,7,8};
    cout<< matrix << endl;
    transform(input, mat,matrix,output);
}
void touying(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, cv::Mat& out)
{
    cout<<"touying" <<endl;
}

void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
               std::vector<float> angle,Eigen::Matrix3f matrix,
               pcl::PointCloud<pcl::PointXYZ>::Ptr & output
               ) {
    Eigen::Matrix4f obbMat = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mat_x = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mat_y = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mat_z = Eigen::Matrix4f::Identity();
    float angle_x = angle[0];
    float angle_y = angle[1];
    float angle_z = angle[2];
    float t1 = angle[3];
    float t2 = angle[4];
    float t3 = angle[5];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            obbMat(i, j) = matrix(i, j);
        }
    }
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//定义平移矩阵，并初始化为单位阵
    translation(0, 3) = t1;
    translation(1, 3) = t2;
    translation(2, 3) = t3;
    pcl::transformPointCloud(*input, *output, translation);
    pcl::transformPointCloud(*output, *output, obbMat);

{
//    cout<< angle_x << endl;
    //    Eigen::Vector4f cloudCentroid;
//    pcl::compute3DCentroid(*cloud, cloudCentroid);//计算点云质心

//
//    // x
////    Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();//定义绕X轴的旋转矩阵，并初始化为单位阵
////    double angle_x = M_PI / 2;//旋转90°
//    mat_x(1, 1) = cos(angle_x);
//    mat_x(1, 2) = -sin(angle_x);
//    mat_x(2, 1) = sin(angle_x);
//    mat_x(2, 2) = cos(angle_x);
//    pcl::transformPointCloud(*output, *output, mat_x);
//
////    Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();//定义绕Y轴的旋转矩阵，并初始化为单位阵
////    double angle_y = M_PI / 2;//旋转90°
//    mat_y(0, 0) = cos(angle_y);
//    mat_y(0, 2) = sin(angle_y);
//    mat_y(2, 0) = -sin(angle_y);
//    mat_y(2, 2) = cos(angle_y);
//    pcl::transformPointCloud(*output, *output, mat_y);
//
////    Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();//定义绕Z轴的旋转矩阵，并初始化为单位阵
////    double angle_z = M_PI / 2;//旋转90°
//    mat_z(0, 0) = cos(angle_z);
//    mat_z(0, 1) = -sin(angle_z);
//    mat_z(1, 0) = sin(angle_z);
//    mat_z(1, 1) = cos(angle_z);
//    pcl::transformPointCloud(*output, *output, mat_z);
}
}

Eigen::Matrix3f getTransformMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
                        std::vector<float> & angle){
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(input);
    feature_extractor.compute();
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
//    pcl::PointXYZ min_point_AABB;
//    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
//    feature_extractor.getAABB(min_point_AABB,max_point_AABB);
    feature_extractor.getOBB(min_point_OBB,max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value,middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector,middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
//    std::cout<< "position_OBB: " << position_OBB << endl;
//    std::cout<< "mass_center: " << mass_center << endl;

    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::AngleAxisf rotation_vector;
    rotation_vector.fromRotationMatrix(rotational_matrix_OBB);
    Eigen::Vector3f eulerAngle=rotation_vector.matrix().eulerAngles(0,1,2);
    std::cout<< eulerAngle<<"HELLO"<<endl;
    for (int i=0; i<3;i++)
    {
        angle.push_back(-eulerAngle[i]);
//        cout<< eulerAngle[i] <<endl;
    }
    angle.push_back(-position_OBB.x);
    angle.push_back(-position_OBB.y);
    angle.push_back(-position_OBB.z);

//    angle.push_back(0);
//    angle.push_back(0);
//    angle.push_back(0);

    return rotational_matrix_OBB.inverse();
//    return rotational_matrix_OBB;

}

void randomTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{


//  Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

  // 定义一个旋转矩阵 (见 https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/4; // 弧度角
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 2.0, 0.0, 0.0;
    transform_2.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*input,*output,transform_2);
    theta = M_PI/4;
    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    transform_3.translation() << 0.0, 0.0, 0.0;
    transform_3.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud(*output,*output,transform_3);



    
}

void ply2pcd()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PolygonMesh mesh;
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::loadPolygonFilePLY("../file/meshed.ply", mesh);
    pcl::io::mesh2vtk(mesh, polydata);
    pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
    pcl::io::savePCDFileASCII("../file/meshed.pcd", *cloud);
}
void shengzhang2()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile <pcl::PointXYZ>("../file/passsta.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
//        return (-1);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(500);//最小的聚类点数
    reg.setMaxClusterSize(10000);//最大的聚类点数
    reg.setSearchMethod(tree);//搜索方式
    reg.setNumberOfNeighbours(30);//设置搜索的邻域点的个数
    reg.setInputCloud(cloud);//输入点
    //reg.setIndices (indices);
    reg.setInputNormals(normals);//输入的法线
    reg.setSmoothnessThreshold(7.5 / 180.0 * M_PI);//设置平滑角度
    reg.setCurvatureThreshold(1.0);//设置曲率阈值

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    //	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //	reg.segment(*clusters, *coefficients);


    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
              std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters[0].indices.size())
    {
        std::cout << clusters[0].indices[counter] << ", ";

        counter++;
        if (counter % 10 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    { //迭代容器中的点云的索引，并且分开保存索引的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            //设置保存点云的属性问题
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "D:/1/0" << j << ".pcd";
//        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        j++;
        *add_cloud += *cloud_cluster;
        //		pcl::io::savePCDFileASCII("add_cloud.pcd", *add_cloud);
    }



    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Ransac"));
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(colored_cloud, "cloud");
    viewer->spin();
//    pcl::visualization::CloudViewer viewer("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//
//    while (!viewer.wasStopped())
//    {
//    }
}

void dbscanFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud_input,
                  std::vector<pcl::PointCloud<pcl::PointXYZ>> & output)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(point_cloud_input);
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANSimpleCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(3);

    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloud_input);
    ec.extract(cluster_indices);

//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
    for (auto it = cluster_indices.begin();
    it != cluster_indices.end();
    ++it)
    {
        std::cout << "size:" <<it->indices.size() <<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        inliers->indices = (*it).indices;
        inliers->header = (*it).header;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // Extract the inliers
        extract.setInputCloud(point_cloud_input);
        extract.setIndices(inliers);
        extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
        // you need to identify the cloud_ as the output of these indices
        extract.filter(*cluster);


        cout<<"dianshu :"<<cluster->points.size()<<endl;
        cluster -> width =  cluster->points.size();
        cluster -> height = 1;
        cluster -> is_dense = true;
        output.push_back(*cluster);
    }
}

void oushi()
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("../file/passsta.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

    pcl::visualization::PCLVisualizer viewer("Cluster Extraction");
    // 注册键盘事件
//    viewer.registerKeyboardCallback(&keyboard_event_occurred, (void*)NULL);
    int v1(1);
    int v2(2);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);

    //创建滤波对象: 使用下采样，叶子的大小为 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

    viewer.addPointCloud(cloud, "cloud1", v1);
    viewer.addPointCloud(cloud_filtered, "cloud2", v2);
    //渲染10秒再继续
    viewer.spinOnce(10000);

    // 创建平面分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    // 把点云中所有的平面全部过滤掉，重复过滤，直到点云数量小于原来的0.3倍
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Write the planar inliers to disk
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);

        //更新显示点云
        viewer.updatePointCloud(cloud_filtered, "cloud1");
        viewer.updatePointCloud(cloud_f, "cloud2");
        //渲染3秒再继续
        viewer.spinOnce(3000);

        cloud_filtered = cloud_f;

    }

    viewer.removePointCloud("cloud2", v2);

    // 创建KdTreee对象作为搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    //聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
    ec.extract(cluster_indices);

    //遍历抽取结果，将其显示并保存
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //创建临时保存点云族的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        //通过下标，逐个填充
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*

        //设置点云属性
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "当前聚类 "<<j<<" 包含的点云数量: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        j++;

        //显示,随机设置不同颜色，以区分不同的聚类
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cloud_cluster, rand()*100 + j * 80, rand() * 50 + j * 90, rand() * 200 + j * 100);
        viewer.addPointCloud(cloud_cluster,cluster_color, ss.str(), v2);
        viewer.spinOnce(5000);
    }
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
void getHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_input,
               float *height)
{
    cout<< "calculate height"<<endl;
    float z_min,z_max;
//    z_min = pcl::getMinMax3D(cloud_input,)
    return;
}

void getOneusingDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr & point_cloud_input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr & output)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ> >  output_vec;
    dbscanFilter(point_cloud_input,output_vec);

    for (auto i = 0;i<output_vec.size();i++)
    {
        if (i==0)
            *output=output_vec.at(0);
        else
        {
            *output+=output_vec.at(i);
        }
    }
}
