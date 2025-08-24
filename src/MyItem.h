//
// Created by sophda on 4/20/23.
//

#ifndef QT_MYITEM_H
#define QT_MYITEM_H
#include <QObject>
#include <QGraphicsItem>
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>
#include "Port.h"
#include "Macro.h"
class MyItem : public QObject, public QGraphicsItem {
    Q_OBJECT

public:
    explicit MyItem(QObject *parent=0);
    void init(int pos);
    void initTitle() ;
    void initPort(int pos);

    QGraphicsTextItem *title;
    std::string title_name;
    int title_height = 10,title_padding = 3;
    QFont title_font;
    QGraphicsEllipseItem* port_node_1,*port_node_2;

    int height=240,width=160;
    int fun_name,class_name;  // while using link
    int index_to_vec=-10;  // while using auto running , this index is used to find the datanode in vec

protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = Q_NULLPTR) override;


private:
    int radius=20;


};


#endif //QT_MYITEM_H
