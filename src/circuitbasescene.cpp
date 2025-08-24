//#include "CircuitBaseScene.h"
#include "circuitbasescene.h"
#include <QtMath>
#include <QPainter>
#include <QDebug>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include <QMetaEnum>
#include <QKeyEvent>
#include <QMimeData>
#include <QDebug>
#include <iostream>

#include "FunctionImage.h"
#include "FuntionCloud.h"

using namespace std;
int CircuitBaseScene::gGridSize = 10;
int CircuitBaseScene::gGridSquares = 5;
QColor CircuitBaseScene::gBackgroundColor = QColor("#393939");
QColor CircuitBaseScene::gForegroundColorDark = QColor("#292929");
QColor CircuitBaseScene::gForegroundColorLight = QColor("#2f2f2f");
CircuitBaseScene::CircuitBaseScene(QObject *parent)
    : QGraphicsScene(parent)
{
    //parent_window = dynamic_cast<mainwindow *>(parent);

}

CircuitBaseScene::~CircuitBaseScene()
{

}

void CircuitBaseScene::keyPressEvent(QKeyEvent *event)
{
    QGraphicsScene::keyPressEvent(event);
}

void CircuitBaseScene::keyReleaseEvent(QKeyEvent *event)
{

}
void CircuitBaseScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
void CircuitBaseScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->setBrush(CircuitBaseScene::gBackgroundColor);
    painter->setPen(CircuitBaseScene::gBackgroundColor);
    painter->drawRect(rect);

    int left = qFloor(rect.left());
    int right = qCeil(rect.right());
    int top = qFloor(rect.top());
    int bottom = qCeil(rect.bottom());

    int _left = left - left % CircuitBaseScene::gGridSize;
    int _top  = top  - top  % CircuitBaseScene::gGridSize; /* 左上角第一个交叉点 */

    QVector<QLine> light_lines, dark_lines;
    int wh = CircuitBaseScene::gGridSize * CircuitBaseScene::gGridSquares;

    for (int i = _left; i <= right; i += CircuitBaseScene::gGridSize)
    {
        if(i % wh == 0)
            dark_lines.append(QLine(i, top, i, bottom));
        else
            light_lines.append(QLine(i, top, i, bottom));
    }

    for (int j = _top; j <= bottom; j += CircuitBaseScene::gGridSize) {
        if(j % wh == 0)
            dark_lines.append(QLine(left, j, right, j));
        else
            light_lines.append(QLine(left, j, right, j));
    }

    painter->setBrush(Qt::NoBrush);
    painter->setPen(CircuitBaseScene::gForegroundColorLight);
    painter->drawLines(light_lines);

    painter->setPen(CircuitBaseScene::gForegroundColorDark);
    painter->drawLines(dark_lines);
}

void CircuitBaseScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void CircuitBaseScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
//    QGraphicsScene::mousePressEvent(mouseEvent);
    if(use_link) {
        auto select_items = this->items(event->scenePos(), Qt::IntersectsItemShape, Qt::DescendingOrder);
        std::cout << "lenght:" << select_items.length() << std::endl;
        if (event->button() == Qt::RightButton && item_select == 0 && select_items.length() == 2) {
            item_ = (MyItem *) select_items.at(1);
            std::cout << "first clicked" << item_->title_name << std::endl;
            port_start = event->scenePos();
            first_click_class = item_->class_name;
            first_click_fun = item_->fun_name;
            item_select = 1;
        } else if (event->button() == Qt::RightButton && item_select == 1 && select_items.length() == 2) {
            item_ = (MyItem *) select_items.at(1);
            std::cout << "first clicked" << item_->title_name << std::endl;
            line = new QGraphicsLineItem(QLineF(port_start, event->scenePos()));
            line->setPen(QPen(QColor(0, 255, 0), 3));
            line->setZValue(-1);
            addItem(line);
            item_select = 0;
//        std::cout<< "2 clicked" <<std::endl;
            second_click_fun = item_->fun_name;
            createLink(first_click_class, first_click_fun);
            select_sum++;
            cout << "selectsum" << select_sum << endl;
        }
    }
    else
    {
        auto select_items = this->items(event->scenePos(), Qt::IntersectsItemShape, Qt::DescendingOrder);
        if (event->button() == Qt::RightButton && item_select == 0 && select_items.length() == 2)
        {
            port_start = event->scenePos();

            item_ = (MyItem *)select_items.at(1);
            run_first = item_->index_to_vec;
            item_select = 1;
            if(run_first==0)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    iter->cloud_xyz = new pcl::PointCloud<pcl::PointXYZ>;
                pcl::io::loadPCDFile("/home/sophda/project/qt/file/points.pcd", *cloud);
                al_vec[0].cloud_xyz = cloud;
            }
        }
        else if (event->button() == Qt::RightButton && item_select == 1 && select_items.length() == 2)
        {
            line = new QGraphicsLineItem(QLineF(port_start, event->scenePos()));
            line->setPen(QPen(QColor(0, 255, 0), 3));
            line->setZValue(-1);
            addItem(line);
            item_ = (MyItem *)select_items.at(1);
            run_second = item_->index_to_vec;

            transDataRuntime(run_first,run_second);
            item_select = 0;
        }


    }
    return QGraphicsScene::mousePressEvent(event);
}


void CircuitBaseScene::createLink(int classname,int fun_name)
{

    tmp = new DataNode();
    if (select_sum==0 && second_click_fun != OUTPUT)
    {
        tmp ->fun_name = DATA;
        tmp ->class_name = POINTCLOUD;
        node_head = tmp;
        node_current = tmp;

        cout<<"head  node !!!!"<<endl;

//        tmp ->child
    }
    else if (second_click_fun != OUTPUT){

        switch(fun_name)
        {
            case ZHITONG:
                tmp -> fun_name = fun_name;
                tmp -> class_name = classname;
                tmp -> node_parent = node_current;
                node_current-> node_child = tmp;
                node_current = tmp;
                break;
            case TONGJI:
                tmp -> fun_name = fun_name;
                tmp -> class_name = classname;
                tmp -> node_parent = node_current;
                node_current-> node_child = tmp;
                node_current = tmp;
                break;
            case FANGZHENG:
                tmp -> fun_name = fun_name;
                tmp -> class_name = classname;
                tmp -> node_parent = node_current;
                node_current-> node_child = tmp;
                node_current = tmp;
                break;
            case TOUYING:
                tmp -> fun_name = fun_name;
                tmp -> class_name = classname;
                tmp -> node_parent = node_current;
                node_current-> node_child = tmp;
                node_current = tmp;
                break;
            case TUOYUAN:
                tmp -> fun_name = fun_name;
                tmp -> class_name = classname;
                tmp -> node_parent = node_current;
                node_current-> node_child = tmp;
                node_current = tmp;
                break;

            default:
                break;
        }
    }
    else if (second_click_fun == OUTPUT)
    {

    }
}

void CircuitBaseScene::scanLink() {
    DataNode *iter;
    std::cout << "ff" << std::endl;
    iter = node_head;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    iter->cloud_xyz = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::io::loadPCDFile("/home/sophda/project/qt/file/leaf.pcd", *cloud);
    iter->cloud_xyz = cloud;
    cout<<iter->cloud_xyz->points.size()<<endl;
//    iter->cloud_xyz = new pcl::PointCloud<pcl::PointXYZ>::Ptr;
    for (int i = 0; i<select_sum && iter!=nullptr ;i++)
    {
        cout<<"iter"<<iter->fun_name<<endl;
        switch (iter->fun_name ) {
            case DATA:
                transData(iter);
                break;
            case ZHITONG:
                passthrough(iter->cloud_xyz,iter->cloud_xyz);
                transData(iter);
                break;
            case TONGJI:
                statistic(iter->cloud_xyz,iter->cloud_xyz);
                transData(iter);
                break;
            case FANGZHENG:
                fangzheng(iter->cloud_xyz,iter->cloud_xyz);
                transData(iter);
                break;
            default:
                break;
        }
//        std::cout<< iter->fun_name <<std::endl;
        iter = iter->node_child;
    }

}

void CircuitBaseScene::transData(DataNode *input) {

    if (input->node_child== nullptr)
    {

    }
    else if(input->node_child!= nullptr)
    {
        input->node_child->cloud_xyz = input->cloud_xyz;
    }

}

void CircuitBaseScene::transDataRuntime(int first,int second) {
    // create a temp var ,which is yinyong of datanode in vector
    DataNode& node_runtime = al_vec.at(second);
    if(node_runtime.class_name == POINTCLOUD)
    {
//        node_runtime.cloud_xyz = new
//      node_runtime.cloud_xyz = al_vec[first].cloud_xyz; copy the pointer to the temp var
        node_runtime.cloud_xyz = al_vec[first].cloud_xyz;
    }
    else
    {

    }

    switch (node_runtime.fun_name)
    {

        case ZHITONG:
            passthrough(node_runtime.cloud_xyz,node_runtime.cloud_xyz);
            break;
        case TONGJI:
            statistic(node_runtime.cloud_xyz,node_runtime.cloud_xyz);
            break;
        case FANGZHENG:
            fangzheng(node_runtime.cloud_xyz,node_runtime.cloud_xyz);
            break;
        case OUTPUT:
            std::cout<<"output"<<std::endl;
            viewer->removeAllPointClouds();
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_handler(al_vec[first].cloud_xyz, 0, 255, 0);
            viewer->addPointCloud(al_vec[first].cloud_xyz,"hh");
            break;
        case JULEI:
            getOneusingDBSCAN(node_runtime.cloud_xyz,node_runtime.cloud_xyz);
            break;
        default:
            break;

    }
}

void CircuitBaseScene::setViewPtr(pcl::visualization::PCLVisualizer::Ptr &viewer_input) {
    viewer = viewer_input;
}

//// second click, on the second port, cloud_tmp->class_name_child receive the next port's input class name
//void CircuitBaseScene::fillLink(int class_name_input, int classname_output, int classname, int fun_name) {
//
//    if (current_data == IMAGE)
//    {
//
//    }
//    else{
//        cloud_tmp->class_name_child = class_name_input;
//
//        // give this tmp pointer to the void* link
//        nodedata_current = cloud_tmp;
//
//        delete cloud_tmp;
//
//    }
//}
