//
// Created by sophda on 4/22/23.
//

#ifndef QT_PORT_H
#define QT_PORT_H
#include <QObject>
#include <QGraphicsItem>
class Port :public QObject, public QGraphicsEllipseItem {
Q_OBJECT
public:
    explicit Port(QObject* parent=0);

protected:
//    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = Q_NULLPTR) override;
    QRectF boundingRect() const override;
};


#endif //QT_PORT_H
