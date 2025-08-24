#ifndef QGRAPHICSBASESCENE_H
#define QGRAPHICSBASESCENE_H

#include <QGraphicsScene>
#include <QKeyEvent>
#include <QGraphicsScene>
#include <QObject>
#include <QLineEdit>
#include <QTextEdit>
#include <QAction>
#include <QGraphicsLineItem>
#include "MyItem.h"
#include "NodeData.h"
#include "FuntionCloud.h"

//#include "../mainwindow.h"
class CircuitBaseScene : public QGraphicsScene
{
public:
    bool use_link = false;
    std::vector<DataNode> al_vec;
    explicit CircuitBaseScene(QObject *parent = nullptr);
    ~CircuitBaseScene();
    void createLink(int classname,int fun_name);
    void scanLink();
    static void transData(DataNode *input);
    void transDataRuntime(int first,int second);

    pcl::visualization::PCLVisualizer::Ptr viewer;
    void setViewPtr(pcl::visualization::PCLVisualizer::Ptr &viewer_input);


//    void fillLink(int class_name_input,int classname_output, int classname,int fun_name);

    static int gGridSize;
    static int gGridSquares;
    static QColor gBackgroundColor;
    static QColor gForegroundColorDark;
    static QColor gForegroundColorLight;
    int item_select=0;
    QPointF port_start,port_end ;
    QGraphicsLineItem* line;
    MyItem* item_;
//    void *nodedata_head,*nodedata_current;
    DataNode* node_head,*node_current,*tmp;
//    mainwindow &parent_window;

    int select_sum=0,first_click_class,first_click_fun,second_click_fun;
    int run_first,run_second;


protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void drawBackground(QPainter *painter, const QRectF &rect);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);
    
};

#endif // QGRAPHICSBASESCENE_H
