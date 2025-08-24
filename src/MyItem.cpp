//
// Created by sophda on 4/20/23.
//

#include "MyItem.h"

// remember to make this defination, or you will face the mistake that the defination cannot be found
// due to the loss of this defination
MyItem::MyItem(QObject *parent) : QObject(parent) {

}

QRectF MyItem::boundingRect() const {
    return QRectF(0,0,width,height);

}
void MyItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    QPainterPath painter_path;
    painter_path.addRoundRect(0,0,width,height,radius);
    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(QColor(100,100,100)));
    painter->drawPath(painter_path);
}

void MyItem::initTitle()  {
    // title(pointer) is the member of Item class,there the title will be given a pointer place
    title = new QGraphicsTextItem(this);
    title->setPlainText(QString::fromStdString(title_name));
    title_font.setFamily("KaiTi");//字体
    title_font.setPointSize(title_height);
    title->setFont(title_font);
    title->setDefaultTextColor(QColor(255,255,255));
    title->setPos(10,10);

}

void MyItem::initPort(int pos) {
    if (pos==-1)
    {
        port_node_1 = new QGraphicsEllipseItem(QRectF(0,0,25,25),this);
        port_node_1->setPen(Qt::NoPen);
        port_node_1->setBrush(QColor(0,2,250));
        port_node_1->setPos(0,int(height/2));

    }
    else if(pos==1){
        port_node_2 = new QGraphicsEllipseItem(QRectF(0,0,25,25),this);

        port_node_2 ->setPen(Qt::NoPen);
        port_node_2->setBrush(QColor(0,2,250));
        port_node_2->setPos(width-25,int(height/2));
    }
    else
    {
        port_node_1 = new QGraphicsEllipseItem(QRectF(0,0,25,25),this);
        port_node_1->setPen(Qt::NoPen);
        port_node_1->setBrush(QColor(0,2,250));
        port_node_1->setPos(0,int(height/2));
        port_node_2 = new QGraphicsEllipseItem(QRectF(0,0,25,25),this);

        port_node_2 ->setPen(Qt::NoPen);
        port_node_2->setBrush(QColor(0,2,250));
        port_node_2->setPos(width-25,int(height/2));

    }

}

void MyItem::init(int pos) {
    initTitle();
    initPort(pos);
}



