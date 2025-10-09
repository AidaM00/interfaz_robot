#include "interfaz_robot.h"
#include <QPixmap>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <ctime> 
#include <QTimer>
#include <QMessageBox>

interfaz_robot::interfaz_robot(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
        camara = new CVideoAcquisition("rtsp://admin:admin@192.168.1.105:8554/profile0");
    // Conectar botones con sus slots
    connect(ui.btnInicio, &QPushButton::clicked, this, &interfaz_robot::startStopCapture);
    connect(ui.btnCapturar, &QPushButton::clicked, this, &interfaz_robot::MostrarImagen);
    connect(ui.btnGuardar, &QPushButton::clicked, this, &interfaz_robot::GuardarImagen);
    connect(ui.btnMover1, &QPushButton::clicked, this, &interfaz_robot::MoverEje);
    connect(ui.btnMoverTodos, &QPushButton::clicked, this, &interfaz_robot::MoverTodosLosEjes);
    // Conexiones para detectar cambios en los spinbox
    connect(ui.spinEje0, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje1, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje2, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje3, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje4, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje5, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);

    // Crear el temporizador para mostrar vídeo en vivo
    timerVideo = new QTimer(this);
    connect(timerVideo, &QTimer::timeout, this, &interfaz_robot::MostrarVideo);

    ui.lblInicio->setText("Vídeo no iniciado");
	m_robot = new Ccom_robot(3); // Poner el n de puerto que sea
}

interfaz_robot::~interfaz_robot()
{
	delete camara;
}

void interfaz_robot::HabilitarBotones(bool habilitar)
{
    ui.btnInicio->setEnabled(habilitar);
    ui.btnCapturar->setEnabled(habilitar);
    ui.btnGuardar->setEnabled(habilitar);
}

void interfaz_robot::startStopCapture()
{
    static bool capturando = false;
    capturando = !capturando;

    if (capturando) {
        camara->startStopCapture(true);  // iniciar cámara
        timerVideo->start(33);           // actualizar cada 33 ms (~30 FPS)
        ui.btnInicio->setText("Detener");
    }
    else {
        timerVideo->stop();              // detener refresco
        camara->startStopCapture(false);
        ui.btnInicio->setText("Iniciar");
    }
}

void interfaz_robot::MostrarVideo()
{
    cv::Mat frame = camara->getImage();
    if (frame.empty()) {
        qDebug() << "Frame vacío recibido";
        return;
    }

    // Guardamos una copia del último frame mostrado
    ultimoFrame = frame.clone();

    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    QImage img((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);

    ui.lblInicio->setPixmap(QPixmap::fromImage(img).scaled(ui.lblInicio->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void interfaz_robot::MostrarImagen()
{
    if (ultimoFrame.empty()) {
        qDebug() << "Error: No hay imagen disponible para capturar.";
        return;
    }

    cv::Mat rgbImg;
    cv::cvtColor(ultimoFrame, rgbImg, cv::COLOR_BGR2RGB);
    QImage qImg((uchar*)rgbImg.data, rgbImg.cols, rgbImg.rows, rgbImg.step, QImage::Format_RGB888);

    ui.lblCaptura->setPixmap(QPixmap::fromImage(qImg).scaled(ui.lblCaptura->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    qDebug() << "Imagen capturada mostrada en lblCaptura.";
}


void interfaz_robot::GuardarImagen()
{
    cv::Mat img = camara->getImage();  // Obtener la imagen actual de la cámara
    if (!img.empty()) {
        // Obtener la fecha y hora actual
        std::time_t t = std::time(nullptr);
        std::tm now;
        localtime_s(&now, &t);  // Usar la versión segura localtime_s

        // Crear nombre de archivo con timestamp: captura_YYYYMMDD_HHMMSS.jpg
        char nombreArchivo[50];
        std::strftime(nombreArchivo, sizeof(nombreArchivo), "captura_%Y%m%d_%H%M%S.jpg", &now);

        cv::imwrite(nombreArchivo, img);  // Guardar la imagen en un archivo
    }
    else {
        qDebug() << "Error: No hay imagen para guardar.";
    }
}

void interfaz_robot::MoverEje()
{
    int eje = ui.comboBoxMover1->currentIndex();
    int grados = ui.spinMover1->value();

    // Validación
    if (grados < -90 || grados > 90) {
        QMessageBox::warning(this, "Error", "El valor de grados debe estar entre -90 y 90.");
        return;
    }

    // Enviar comando al robot
    if (m_robot) {
        qDebug() << "Moviendo eje" << eje << "a" << grados << "grados.";
        m_robot->mover(eje, grados);
        QMessageBox::information(this, "Movimiento",
            QString("Eje %1 movido a %2 grados").arg(eje).arg(grados));
    }
    else {
        QMessageBox::critical(this, "Error", "Robot no inicializado.");
    }

    // Reiniciar spinbox y combo
    ui.spinMover1->setValue(0);
    ui.comboBoxMover1->setCurrentIndex(0);

}

void interfaz_robot::MoverTodosLosEjes()
{
    // Leer los valores actuales de los spinbox
    int angulos[6] = {
        ui.spinEje0->value(),
        ui.spinEje1->value(),
        ui.spinEje2->value(),
        ui.spinEje3->value(),
        ui.spinEje4->value(),
        ui.spinEje5->value()
    };

    // Verificar que todos los valores estén dentro del rango permitido
    for (int i = 0; i < 6; ++i)
    {
        if (angulos[i] < -90 || angulos[i] > 90)
        {
            QMessageBox::warning(this, "Valor fuera de rango",
                QString("El eje %1 tiene un valor no válido (%2°).\n\n"
                    "Debe estar entre -90° y +90°.")
                .arg(i).arg(angulos[i]));
            return; // Cancelar el envío
        }
    }

    // Construir el comando tipo: #a-45-0-30-90--30-60*
    QString comando = "#a";
    for (int i = 0; i < 6; ++i) {
        comando += QString::number(angulos[i]);
        if (i < 5) comando += "-";
    }
    comando += "*";

    // Enviar el comando al robot
    if (m_robot)
        m_robot->enviarComando(comando);

    qDebug() << "Comando enviado al robot:" << comando;

    QMessageBox::information(this, "Movimiento",
        "Todos los ejes se están moviendo simultáneamente a las nuevas posiciones.");

    // Reiniciar todos los spinbox
    ui.spinEje0->setValue(0);
    ui.spinEje1->setValue(0);
    ui.spinEje2->setValue(0);
    ui.spinEje3->setValue(0);
    ui.spinEje4->setValue(0);
    ui.spinEje5->setValue(0);

}

void interfaz_robot::VerificarRango(int valor)
{
    QSpinBox* spin = qobject_cast<QSpinBox*>(sender()); // Saber cuál spinbox cambió
    if (!spin) return;

    // Verificar si está fuera de rango
    if (valor < -90 || valor > 90) {
        spin->setStyleSheet("color: red; font-weight: bold;");
    }
    else {
        spin->setStyleSheet("color: black;");
    }
}





//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar la QLabel
//}

