//
// Created by sophda on 4/20/23.
//

#include "MyView.h"
MyView::MyView(QGraphicsScene *scene, QWidget *parent)
    :QGraphicsView(parent)  // init the parent
{
    this->setScene(scene);
}

void MyView::init() {
    this->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform | QPainter::TextAntialiasing
                         | QPainter::HighQualityAntialiasing );
    this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setDragMode(QGraphicsView::ScrollHandDrag);
    this->setContextMenuPolicy(Qt::CustomContextMenu);
}

MyView::MyView(QWidget *parent)  {
    init();

}


void MyView::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? scale(1.05,1.05) : scale(0.95,0.95);
}

void MyView::mousePressEvent(QMouseEvent *event)
{
    QGraphicsView::mousePressEvent(event);
    if(this->scene() == nullptr)
    {
//        qDebug() << "The scene is null";
        std::cout<<"scene is null"<<std::endl;
        return;
    }
    // 记录鼠标按下时的中心点坐标
    centerAnchor = mapToScene(event->pos()) - event->pos() + QPointF(width() / 2, height() / 2);
    // 记录当前鼠标在view中的位置，用来在mouseMove事件中计算偏移
    // 此处不将view坐标转换成scene坐标的原因是优化性能，在move的过程中会产生抖动
    posAnchor = event->pos();
    isMousePressed = true;
//    std::cout<<centerAnchor.x()<<std::endl;
}

void MyView::mouseMoveEvent(QMouseEvent *event)
{
    QGraphicsView::mouseMoveEvent(event);
    QPointF offsetPos = event->pos() - posAnchor;
    if(isMousePressed){
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        centerOn(centerAnchor - offsetPos);
    }
}

void MyView::mouseReleaseEvent(QMouseEvent *event)
{
    QGraphicsView::mouseReleaseEvent(event);
    isMousePressed = false;
}


