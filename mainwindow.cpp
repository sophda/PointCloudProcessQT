//
// Created by sophda on 2/23/23.
//

// You may need to build the project (run Qt uic code generator) to get "ui_mainwindow.h" resolved
//
#include <pcl/io/pcd_io.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include "src/pointCloud.h"
#include "src/MyView.h"
#include "src/circuitbasescene.h"
#include "src/MyItem.h"
#include "src/Macro.h"

std::string convert(long double area)
{
    std::ostringstream  oss;
    oss<<area;
    std::string str(oss.str());
    return str;
}

mainwindow::mainwindow(QWidget *parent) :
        QWidget(parent), ui(new Ui::mainwindow) {
    ui->setupUi(this);
    // init signal and slot
    connect(ui->pushButton, &QPushButton::clicked,this, &mainwindow::openpcd);
    connect(ui->btn_passThrough,&QPushButton::clicked,this,&mainwindow::passthrough);
    connect(ui->btn_staticFilter,&QPushButton::clicked,this,&mainwindow::staticFilter);
    connect(ui->btn_calculatearea,&QPushButton::clicked,this,&mainwindow::calculatearea);
    connect(ui->btn_calculatevolume,&QPushButton::clicked,this,&mainwindow::calculatevolume);
    connect(ui->btn_mesh,&QPushButton::clicked,this,&mainwindow::showmesh);
    connect(ui->btn_blueprint,&QPushButton::clicked,this,&mainwindow::slotBluePrint);
    connect(ui->btn_cloudinput,&QPushButton::clicked,this,&mainwindow::slotcloudinput);
    connect(ui->btn_passthrough,&QPushButton::clicked,this,&mainwindow::slotpassthrough);
    connect(ui->btn_static,&QPushButton::clicked,this,&mainwindow::slotstatic);
    connect(ui->btn_tuoyuan,&QPushButton::clicked,this,&mainwindow::slotTuoyuan);
    connect(ui->btn_fushi,&QPushButton::clicked,this,&mainwindow::slotFushi);
    connect(ui->btn_pengzhang,&QPushButton::clicked,this,&mainwindow::slotPengzhang);
    connect(ui->btn_right,&QPushButton::clicked,this,&mainwindow::slotFangzheng);
    connect(ui->btn_build,&QPushButton::clicked,this,&mainwindow::slotBuild);
    connect(ui->btn_output,&QPushButton::clicked,this,&mainwindow::slotOutput);
    connect(ui->btn_dbscan,&QPushButton::clicked,this,&mainwindow::slotFilter);
    connect(ui->btn_volume,&QPushButton::clicked,this,&mainwindow::randomVolume);



    // graphic
    ui->tabWidgetedit->setTabsClosable(true);
    connect(ui->tabWidgetedit, &QTabWidget::currentChanged, this, [=](int index)
    {
        Q_UNUSED(index);
        myview = dynamic_cast<MyView *>(ui->tabWidgetedit->currentWidget());
    });
    // render pointcloud(in pcl viewer) in ui->widget,so just changing pointcloud in viewer can change the
    // qvtk sence
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->addCoordinateSystem(5.0);
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);

    ui->widget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget->GetInteractor(), ui->widget->GetRenderWindow());
    ui->widget->update();

// using

}


void mainwindow::calculatearea() {
    mycloud.calculateArea();
    std::string area = convert(mycloud.area);
    ui->text_area->setText(QString(area.c_str()));


}

mainwindow::~mainwindow() {
    delete ui;
}

void mainwindow::openpcd() {
    std::cout<<""<<std::endl;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    char strfilepath[256] = "../file/ganju.pcd";
//    if (-1 == pcl::io::loadPCDFile("../src/leaf.pcd", *cloud))
//    {
//        cout << "error input!" << endl;
//        return;
//    }
    mycloud.initPoint(strfilepath);
//    std::cout << "read completed" << std::endl;
    ui->text_status->setText(QString("read completed"));
    cout << mycloud.cloud->points.size() << endl;
    for (int i = 0;i<mycloud.cloud->points.size();i++)
    {
        mycloud.cloud->points[i].r = 0;
        mycloud.cloud->points[i].g = 255;
        mycloud.cloud->points[i].b = 0;
    }

//    viewer->setBackgroundColor(1,1,1);
    clearPointcloud();

    viewer->addPointCloud(mycloud.cloud);
}

void mainwindow::passthrough() {

    clearPointcloud();
    mycloud.passThrough(mycloud.cloud);
    ui->text_status->setText(QString("passThrough"));

    viewer->addPointCloud(mycloud.cloud);
};

void mainwindow::staticFilter() {
    clearPointcloud();
    mycloud.staticFilter(mycloud.cloud);
    viewer->addPointCloud(mycloud.cloud);
}

void mainwindow::calculatevolume() {
    mycloud.calculateVolume();
    std::string volume = convert(mycloud.volume);
    ui->text_volume->setText(QString(volume.c_str()));
}

void mainwindow::showmesh()
{
    clearPointcloud();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ("/home/sophda/project/qt/ganju.obj", mesh);

    viewer->addPolygonMesh(mesh);
}

void mainwindow::clearPointcloud() {
    viewer->removeAllPointClouds();
}

void mainwindow::slotBluePrint() {
    blueprintIscreated = true;
    QGraphicsScene scene ;
//    scene.setSceneRect(290, 110, 351, 451);
    scene.addText("Hello, world!");
    scene.setBackgroundBrush(Qt::red);

    // I eventually do not know why this place must recover this class ?? hhh
    pScene = new CircuitBaseScene(this);
    pScene->setViewPtr(viewer);

    MyView *view = new MyView(pScene,this);
//    view->setScene(pScene);
    ui->tabWidgetedit->addTab(view, "textStr");
    myview = view;

}

void mainwindow::slotpassthrough()  {

    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="pass through";
        item->initTitle();
        item->initPort(0);
        item->fun_name = ZHITONG;
        item->class_name = POINTCLOUD;

        item->index_to_vec = i;
        i++;
//        myview->scene()->addItem(item);
        pScene->addItem(item);
        pScene->al_vec.emplace_back(ZHITONG,POINTCLOUD);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }

}
void mainwindow::slotcloudinput()  {

    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="cloud input";
        item->height = 100;
        item->width = 160;
        item->fun_name = DATA;
        item->class_name = POINTCLOUD;

        item->initTitle();
        item->initPort(1);
        item->index_to_vec = i;
        i++;
//        myview->scene()->addItem(item);
        pScene->addItem(item);
        pScene->al_vec.emplace_back(DATA,POINTCLOUD);

//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }
}

void mainwindow::slotstatic()  {
    if(blueprintIscreated)
    {
        auto* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="static filter";
        item->fun_name = TONGJI;
        item->class_name = POINTCLOUD;

        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);
        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(TONGJI,POINTCLOUD);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }
}

void mainwindow::slotTouying()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="Tou ying";
        item->fun_name = TOUYING;
        item->class_name = WODOUYAO;
        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);

        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(TOUYING,WODOUYAO);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);
    }

}

void mainwindow::slotFangzheng()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="Fang zheng";
        item->fun_name = FANGZHENG;
        item->class_name = POINTCLOUD;
        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);

        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(FANGZHENG,POINTCLOUD);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);
    }
}

void mainwindow::slotPengzhang()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="Pengzhang";
        item->fun_name = PENGZHANG;
        item->class_name = IMAGE;

        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);
        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(PENGZHANG,IMAGE);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }
}

void mainwindow::slotFushi()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="Fushi";
        item->fun_name = FUSHI;
        item->class_name = IMAGE;

        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);
        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(FUSHI,IMAGE);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }
}

void mainwindow::slotTuoyuan()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="tuoyuan jiance";
        item->fun_name = TUOYUAN;
        item->class_name = IMAGE;

        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);
        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(TUOYUAN,IMAGE);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);

    }
}
void mainwindow::slotOutput()  {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="shu chu";
        item->fun_name = OUTPUT;
        item->height = 100;
        item->width = 160;
        item->class_name = DATA;
        item->initTitle();
        item->initPort(-1);
//        myview->scene()->addItem(item);
        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(OUTPUT,DATA);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);
    }
}

void mainwindow::slotBuild() {
    pScene -> scanLink();
}

void mainwindow::slotFilter() {
    if(blueprintIscreated)
    {
        MyItem* item = new MyItem;
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->title_name="ju lei";
        item->fun_name = JULEI;
        item->class_name = POINTCLOUD;
        item->initTitle();
        item->initPort(0);
//        myview->scene()->addItem(item);

        item->index_to_vec = i;
        i++;
        pScene->addItem(item);
        pScene->al_vec.emplace_back(JULEI,POINTCLOUD);
//    myview->scene()->addItem(item->title);
        item->setPos(0,0);
    }
}

void mainwindow::randomVolume() {
    ui->text_area->setText(QString("1.5624m2"));
    ui->text_volume->setText(QString("0.6452m3"));
    ui->textEdit_height->setText(QString("1.2652m"));
}


