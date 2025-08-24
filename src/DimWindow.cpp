//
// Created by sophda on 5/18/23.
//

#include "DimWindow.h"
#include "threedim.h"
using namespace QtDataVisualization;

DimWindow::DimWindow(QWidget *parent)
:QWidget(parent),ui(new Ui::Form)
{
    ui->setupUi(this);

    QSurface3DSeries *series;
    Q3DSurface *groupBox;
    QSurfaceDataArray * dataArray;

    groupBox = new Q3DSurface;
    groupBox->axisX()->setTitle("X axis");
    groupBox->axisY()->setTitle("Y axis");
    groupBox->axisZ()->setTitle("Z axis");
    groupBox->axisY()->setRange(-5,10);
    groupBox->axisX()->setRange(-11,11);
    groupBox->axisZ()->setRange(-11,11);

    //将三维图与数据进行绑定
    QSurfaceDataProxy * proxy = new QSurfaceDataProxy;
    series = new QSurface3DSeries(proxy);
    /// 这个就已经将数据和图绑定了，后边更改图数据时候，直接改series即可
    groupBox->addSeries(series);

    QVBoxLayout* vLayout = new QVBoxLayout();
    // 还有这个地方，是将groupBox和vLayout绑定到一起，直接改groupBox,页面里面的三维图也会跟着改变的
    vLayout->addWidget(createWindowContainer(groupBox));
    ui->widget->setLayout(vLayout);
    dataArray = new QSurfaceDataArray;

}