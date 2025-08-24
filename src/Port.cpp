//
// Created by sophda on 4/22/23.
//

#include <QPen>
#include "Port.h"

Port::Port(QObject *parent) : QObject(parent) {

}


QRectF Port::boundingRect() const {
    return QRectF(0,0,20,20);
}
