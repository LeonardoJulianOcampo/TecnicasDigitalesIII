#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMenu>
#include <QMenuBar>
#include <QVBoxLayout>
#include <QLCDNumber>
#include <QSlider>
#include <QMessageBox>

#include <QSerialPort>
#include <QSerialPortInfo>
#include <string>
#include <QDebug>

#include <iostream>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    esp32 = new QSerialPort(this);
    resetDisplays(); //para inicializar los display con las rayitas

    //Conexiones de los widgets
    connect(ui->Out_A1_slider, &QSlider::valueChanged, this, &MainWindow::readSlider);
    connect(ui->Out_A2_slider, &QSlider::valueChanged, this, &MainWindow::readSlider);
    connect(ui->outB1, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB2, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB3, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB4, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB5, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB6, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB7, &QRadioButton::toggled, this, &MainWindow::readButton);
    connect(ui->outB8, &QRadioButton::toggled, this, &MainWindow::readButton);


    //para guardar los valores de los widgets en el array
    arrayWidgets();

    //-------------------Código para de la conexión de la placa-------------------------------

    qDebug() << "Number of ports: " << QSerialPortInfo::availablePorts().length() << "\n";
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        qDebug() << "Description: " << serialPortInfo.description() << "\n";
        qDebug() << "Has vendor id?: " << serialPortInfo.hasVendorIdentifier() << "\n";
        qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier() << "\n";
        qDebug() << "Has product id?: " << serialPortInfo.hasProductIdentifier() << "\n";
        qDebug() << "Product ID: " << serialPortInfo.productIdentifier() << "\n";
    }

    //Indetify the port the esp32 is on
    bool esp32_is_available = false;
    QString esp32_port_name;

    //For each available serial port
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        if(serialPortInfo.hasProductIdentifier() && serialPortInfo.hasVendorIdentifier()){
            //check if the product ID and vendor ID match those of the esp32
            if(serialPortInfo.productIdentifier() == esp32_product_id &&
                    serialPortInfo.vendorIdentifier() == esp32_vendor_id){
                esp32_is_available = true; // Esp32 is available on this port
                esp32_port_name = serialPortInfo.portName();
            }
        }
    }
    qDebug() << "La esp32 esta en el puerto: " << esp32_port_name << "\n";


    //Open and configure the arduino port if available
    if(esp32_is_available){
        qDebug() << "Found the arduino port ... \n";
        esp32->setPortName(esp32_port_name);
        esp32->open(QSerialPort::ReadWrite);
        esp32->setBaudRate(QSerialPort::Baud9600);
        esp32->setDataBits(QSerialPort::Data8);
        esp32->setFlowControl(QSerialPort::NoFlowControl);
        esp32->setParity(QSerialPort::NoParity);
        esp32->setStopBits(QSerialPort::OneStop);
        QObject::connect(esp32, SIGNAL(readyRead()), this, SLOT(readSerial()));
    }else{
        qDebug() << "Couldn't find the correct port for the esp32. \n";
        QMessageBox::information(this, "Serial Port Error", "Couldn't open serial port to esp32"); //te abre un mensaje mostrandote el mensaje, lo primero es el titulo, lo segundo es el contenido del mensaje
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

//Lee los datos enviados desde la placa
void MainWindow::readSerial(void)
{
    serialData = esp32->readAll();
    qDebug() << serialData;
    if(serialData.size() == 39 && serialData.at(0) == 'f'){ //debería ser una and no una or
        dato.clear();
        dato = serialData;
        qDebug() << dato.size();
        printData(dato); //llamamos a la función que imprime los datos en los widgets
    }
}

//Esta función configura los displays para que muestren "-" al inciar el programa
void MainWindow::initDisplays(QVBoxLayout *LayoutD)
{
    //Para iterar entre los labels que contiene el layout
    for(int i=0; i<LayoutD->count(); i++){

        //Obtener el elemento en la posición i
        QObject *obj = LayoutD->itemAt(i)->widget();

        //Verificar si el elemento es un QLCDNumber
        if(QLCDNumber *Display = qobject_cast<QLCDNumber*>(obj)){
            Display->display("-");
        }
    }
}

//Le pasa los Layout a la función initDisplay así le pone el símbolo de '-'
void MainWindow::resetDisplays(void)
{
    initDisplays(ui->In_Avalues);
    initDisplays(ui->In_Dvalues);
    initDisplays(ui->Out_Avalues);
    initDisplays(ui->Out_Dvalues);
}

//se ejecuta cuando se muenven los sliders
void MainWindow::readSlider(void)
{

    int value1 = ui->Out_A1_slider->sliderPosition();
    int value2 = ui->Out_A2_slider->sliderPosition();
    intsToByte(value1);
    intsToByte(value2);
    /*
     *Por el momento le estamos mandando números del 0 al 500 dependiendo los valores
     *dependiendo los valores que veamos que admite el pwm ajustamos el slider ahí
     *y después ajustamos los valores de mapeo para que muestre esos valores como de 0 a 5V
    */
    sendData();

}

//Muestra los datos en los widgets
void MainWindow::printData(QByteArray dato)
{
    if(!dato.isEmpty()){

        int j = 2;
        //Asigno los valores desde datos al array con los nombres de los widgets
        for (int i=0; i<26; i++){
            if(i<10){
                if (i>=8 && i<10){
                    channelsWidgets[i].value = mapToVoltage(bytesToInt(dato,(j)));
                }else
                    channelsWidgets[i].value = bytesToInt(dato,(j)); //asigno señales analógicos con 2 bytes
                j+= 2;
            }else{
                int k = i+12;
                channelsWidgets[i].value = dato.at(k); //asigno señales digitales con 1 byte
            }
        }

        //Muestros los valores guardados en los widgets
        for (int i=0; i<26; i++){
            if (channelsWidgets[i].widget) {
                channelsWidgets[i].widget->display(channelsWidgets[i].value);
            } else {
                qDebug() << "El puntero a QLCDNumber no es válido.";
            }
        }
    }

}

//Convierte los datos recibidos de bytes a numero entero (int)
int MainWindow::bytesToInt (const QByteArray &bytes, int i)
{
    QByteArray number;
    number.append(bytes.at(i));
    number.append(bytes.at(i+1));
    int result = 0;
    for(int j=1 ; j>=0; j --){
        result |= (static_cast<unsigned char>(number.at(j)) << ((j)*8));
    }
    qDebug() <<"este es el resultado: " << result;
    return result;
}

//Mapea los valores de los sliders para mostrar de 0a 5v
double MainWindow::mapToVoltage(int value) {
    int minValue = 0, maxValue = 4095;
    double minVoltage = 0.0, maxVoltage = 5.0;

    value = std::max(minValue, std::min(maxValue, value));
    //qDebug() << "este es el valor: " << value;
    double voltage = minVoltage + (maxVoltage - minVoltage) * (static_cast<double>(value - minValue) / (maxValue - minValue));
    //qDebug() << "Este es el valor de volatje antes del redondeo: " << voltage;
    voltage = std::round(voltage * 100.0) / 100.0;
    //qDebug() << "Este es el valor del voltaje después del redondeo: " << voltage;
    return voltage;
}

//Almacena los widgets para su mejor manejo
void MainWindow::arrayWidgets(void){

    //entradas analógicas
    channelsWidgets[0].widget = ui->In_A1;
    channelsWidgets[1].widget = ui->In_A2;
    channelsWidgets[2].widget = ui->In_A3;
    channelsWidgets[3].widget = ui->In_A4;
    channelsWidgets[4].widget = ui->In_A5;
    channelsWidgets[5].widget = ui->In_A6;
    channelsWidgets[6].widget = ui->In_A7;
    channelsWidgets[7].widget = ui->In_A8;

    //Salidas analógicas
    channelsWidgets[8].widget = ui->Out_A1_value;
    channelsWidgets[9].widget = ui->Out_A2_value;

    //Entradas digitales
    channelsWidgets[11].widget = ui->In_D1;
    channelsWidgets[10].widget = ui->In_D2;
    channelsWidgets[12].widget = ui->In_D3;
    channelsWidgets[13].widget = ui->In_D4;
    channelsWidgets[14].widget = ui->In_D5;
    channelsWidgets[15].widget = ui->In_D6;
    channelsWidgets[16].widget = ui->In_D7;
    channelsWidgets[17].widget = ui->In_D8;

    //Salidas digitales
    channelsWidgets[18].widget = ui->Out_D1;
    channelsWidgets[19].widget = ui->Out_D2;
    channelsWidgets[20].widget = ui->Out_D3;
    channelsWidgets[21].widget = ui->Out_D4;
    channelsWidgets[22].widget = ui->Out_D5;
    channelsWidgets[23].widget = ui->Out_D6;
    channelsWidgets[24].widget = ui->Out_D7;
    channelsWidgets[25].widget = ui->Out_D8;
}

//Envía los datos de las salidas
void MainWindow::sendData (void){
    dataToSend[0] = 'I';
    dataToSend[13] = 'E';

    //transformo el array de unsigned char a tipo QByteArray
    QByteArray byteArray(reinterpret_cast<const char*>(dataToSend), sizeof(dataToSend));

    // Escribir datos en el puerto serie
    qint64 bytesWritten = esp32->write(byteArray);
    if (bytesWritten == -1) {
        qDebug() << "Error al escribir en el puerto serie:" << esp32->errorString();
    } else {
        qDebug() << "Bytes escritos:" << bytesWritten;
    }
}

//Para guardar en el array a enviar los valores de las salidas digitales
void MainWindow::readButton(void){
    dataToSend[1] = ui->outB1->isChecked() ? 1 : 0;
    dataToSend[2] = ui->outB2->isChecked() ? 1 : 0;
    dataToSend[3] = ui->outB3->isChecked() ? 1 : 0;
    dataToSend[4] = ui->outB4->isChecked() ? 1 : 0;
    dataToSend[5] = ui->outB5->isChecked() ? 1 : 0;
    dataToSend[6] = ui->outB6->isChecked() ? 1 : 0;
    dataToSend[7] = ui->outB7->isChecked() ? 1 : 0;
    dataToSend[8] = ui->outB8->isChecked() ? 1 : 0;
    sendData();
}

//Para transformar el número de entero a byte
void MainWindow::intsToByte (int number){
    int k = 9;
    for (int i=0; i<2; i++){
        for (int j=0; j<2; j++){
            dataToSend[k] = (number >> (j * 8)) & 0xFF;
            // Serial.println(sendData[k]);
            k++;
        }
    }
}
