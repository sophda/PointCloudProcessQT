//
// Created by sophda on 4/20/23.
//

#ifndef QT_MYVIEW_H
#define QT_MYVIEW_H
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QMenu>
#include <QColorDialog>
#include <iostream>
class MyView : public QGraphicsView {
public:
    explicit MyView(QWidget *parent = nullptr);
    explicit MyView(QGraphicsScene *scene, QWidget *parent);
//    ~MyView();
    bool isMousePressed = false;
    void init();
    QPointF centerAnchor,posAnchor;

protected:

    virtual void wheelEvent(QWheelEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);



};


#endif //QT_MYVIEW_H
