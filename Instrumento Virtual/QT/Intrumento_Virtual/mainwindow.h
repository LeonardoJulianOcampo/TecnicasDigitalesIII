#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QtSerialPort>
#include <QVBoxLayout>
#include <QLCDNumber>
#include <QByteArray>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    typedef struct{
        QLCDNumber *widget;
        int value;

    }configWidget;

    configWidget channelsWidgets[26];
    QByteArray dataToSend;

private slots:
    void readSerial(void);
    void sendData (void);
    void arrayWidgets(void);
    void resetDisplays(void);
    void initDisplays(QVBoxLayout *LayoutD);
    void printData(QByteArray dato);
    void readSlider(void);
    void readButton(void);

private:
    Ui::MainWindow *ui;
    QSerialPort *esp32;
    QTimer *miTimer;
    static const quint16 esp32_vendor_id =  6790 ;//6790;4292
    static const quint16 esp32_product_id = 29987; //21972;//;60000


    QByteArray serialData;
    QByteArray dato;
    //unsigned char dataToSend[14] = {0};

    int bytesToInt (const QByteArray &bytes, int i);
    void intsToByte (int number, int k);
    double mapToVoltage(int value);
    //widgetConfig widgetConfigInstance;  // Declarar una instancia de widgetConfig

};
#endif // MAINWINDOW_H
