#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    QRect screenrect = a.primaryScreen()->geometry();
    w.move(screenrect.left(), screenrect.top()-10);

    w.show();

    return a.exec();
}
