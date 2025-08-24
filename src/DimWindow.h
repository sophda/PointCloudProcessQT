//
// Created by sophda on 5/18/23.
//

#ifndef QT_DIMWINDOW_H
#define QT_DIMWINDOW_H

#include <QWidget>
#include <QtDataVisualization>
#include <QHBoxLayout>
#include <QVBoxLayout>

//using namespace QtDataVisualization;

QT_BEGIN_NAMESPACE
namespace Ui { class Form; }
QT_END_NAMESPACE
class DimWindow : public QWidget
{
public:
    DimWindow(QWidget *parent= nullptr);
private:
    Ui::Form *ui;
};

#endif //QT_DIMWINDOW_H
