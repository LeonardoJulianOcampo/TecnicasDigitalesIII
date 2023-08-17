#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <QDebug>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial= new QSerialPort();
    serial->setPortName("COM9");
    serial->open(QSerialPort::ReadOnly);
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->open(QIODevice::ReadWrite);
    connect(serial, SIGNAL(readyRead()), this, SLOT(RecibirArreglo()));

    if(serial->isOpen())
    {
        qDebug() <<"serial is connected.";
    }
    else
    {
       qDebug () <<"serial in not connected.";
    }
}
void MainWindow::RecibirArreglo()
{
    QByteArray arreglo;
    arreglo=serial->readLine();
     cadena=cadena+arreglo;
    /*if(cadena.length()<4){*/
if(cadena.contains("9")){
        ui->plainTextEdit->appendPlainText(cadena);
        cadena="";
}
}
MainWindow::~MainWindow()
{
    delete ui;
    serial->close();
}




