//
// Created by sophda on 2/23/23.
//

#ifndef QT_MAINWINDOW_H
#define QT_MAINWINDOW_H

#include <QWidget>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkRenderWindow.h>
#include "src/pointCloud.h"
#include "src/MyView.h"
#include "src/circuitbasescene.h"
QT_BEGIN_NAMESPACE
namespace Ui { class mainwindow; }
QT_END_NAMESPACE

class mainwindow : public QWidget {
Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = nullptr);
    MyView *myview;
    CircuitBaseScene *pScene;
    ~mainwindow() override;
    pointCloud mycloud;// while mainwindow class is built , the pointcloud will be intialized
    int i = 0; // item num
    pcl::visualization::PCLVisualizer::Ptr viewer;
private:
    Ui::mainwindow *ui;
    void openpcd();
    void passthrough();
    void staticFilter();
    void calculatearea();
    void calculatevolume();
    void clearPointcloud();
    void showmesh();
    void randomVolume();


    void slotBluePrint();
    // add node fun
    void slotcloudinput() ;
    void slotpassthrough() ;
    void slotstatic() ;
    void slotTouying() ;
    void slotFangzheng() ;
    void slotPengzhang() ;
    void slotFushi() ;
    void slotTuoyuan() ;
    void slotBuild() ;
    void slotOutput() ;
    void slotFilter();




    bool blueprintIscreated=false;
};

#endif //QT_MAINWINDOW_H
