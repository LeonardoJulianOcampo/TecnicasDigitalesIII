#include "mainwindow.h"

#include <QApplication>
#include <QToolBar>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setFixedSize(1141,646);
    w.setWindowTitle("Instrumento Virtual");
    w.show();

    return a.exec();
}
