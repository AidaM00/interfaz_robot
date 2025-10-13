#include "interfaz_robot.h"
#include <QPixmap>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <ctime> 
#include <QTimer>
#include <QMessageBox>
#include <QSpinBox>
#include <cmath>
#include <array>
#include <QString>

interfaz_robot::interfaz_robot(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    camara = new CVideoAcquisition(0);
    // Conectar botones con sus slots
    connect(ui.btnInicio, &QPushButton::clicked, this, &interfaz_robot::startStopCapture);
    connect(ui.btnGuardar, &QPushButton::clicked, this, &interfaz_robot::GuardarImagen);
    connect(ui.btnMover1, &QPushButton::clicked, this, &interfaz_robot::MoverEje);
    connect(ui.btnMoverTodos, &QPushButton::clicked, this, &interfaz_robot::MoverTodosLosEjes);
    connect(ui.btnComunicacionrobot, SIGNAL(clicked()), this, SLOT(iniciarComRobot()));
    
    // Conexiones para detectar cambios en los spinbox
    connect(ui.spinEje0, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje1, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje2, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje3, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje4, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);
    connect(ui.spinEje5, qOverload<int>(&QSpinBox::valueChanged), this, &interfaz_robot::VerificarRango);

    // Crear el temporizador para mostrar v�deo en vivo
    timerVideo = new QTimer(this);
    connect(timerVideo, &QTimer::timeout, this, &interfaz_robot::MostrarVideo);

    ui.lblInicio->setText("V�deo no iniciado");
    ui.lblPosicionActual->setText("Posici�n actual: desconocida");
	HabilitarBotones(false);
 
}

interfaz_robot::~interfaz_robot()
{
	delete camara;
}
void interfaz_robot::iniciarComRobot() 
{
    int com = ui.spinBoxCOM->value();
    m_robot = new Ccom_robot(com);
    HabilitarBotones(true);
    ui.btnComunicacionrobot->setEnabled(false);
}

void interfaz_robot::HabilitarBotones(bool habilitar)
{
    ui.btnInicio->setEnabled(habilitar);
    ui.btnGuardar->setEnabled(habilitar);
    ui.btnMover1 -> setEnabled(habilitar);
    ui.btnMoverTodos->setEnabled(habilitar);
}

void interfaz_robot::startStopCapture()
{
    static bool capturando = false;
    capturando = !capturando;

    if (capturando) {
        camara->startStopCapture(true);  // iniciar c�mara
        timerVideo->start(33);           // actualizar cada 33 ms (unos 30 FPS)
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
        qDebug() << "Frame vac�o recibido";
        return;
    }

    // Guardamos una copia del �ltimo frame mostrado
    ultimoFrame = frame.clone();

    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    QImage img((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);

    ui.lblInicio->setPixmap(QPixmap::fromImage(img));
}

void interfaz_robot::GuardarImagen()
{
    cv::Mat img = camara->getImage();  // Obtener la imagen actual de la c�mara
    if (!img.empty()) {
        // Obtener la fecha y hora actual
        std::time_t t = std::time(nullptr);
        std::tm now;
        localtime_s(&now, &t);  // Usar la versi�n segura localtime_s

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

    // Validaci�n
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
        Directa();  // Calcula y muestra la posici�n resultante
    }
    else {
        QMessageBox::critical(this, "Error", "Robot no inicializado.");
    }

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

    // Verificar que todos los valores est�n dentro del rango permitido
    for (int i = 0; i < 6; ++i)
    {
        if (angulos[i] < -90 || angulos[i] > 90)
        {
            QMessageBox::warning(this, "Valor fuera de rango",
                QString("El eje %1 tiene un valor no v�lido (%2�).\n\n"
                    "Debe estar entre -90� y +90�.")
                .arg(i).arg(angulos[i]));
            return; // Cancelar el env�o
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
        "Todos los ejes se est�n moviendo simult�neamente a las nuevas posiciones.");

    Directa();  // Calcula y muestra la posici�n resultante
}

void interfaz_robot::VerificarRango(int valor)
{
    QSpinBox* spin = qobject_cast<QSpinBox*>(sender()); // Saber cu�l spinbox cambi�
    if (!spin) return;

    // Verificar si est� fuera de rango
    if (valor < -90 || valor > 90) {
        spin->setStyleSheet("color: red; font-weight: bold;");
    }
    else {
        spin->setStyleSheet("color: black;");
    }
}

//void interfaz_robot::ActualizarPosicionRobot()
//{
//    if (!m_robot) {
//        ui.lblPosicionActual->setText("Robot no conectado");
//        return;
//    }
//    
//    std::vector<int> angulos = m_robot->obtenerPosicionActual();
//
//    if (angulos.size() == 6) {
//        QString texto = QString("Posici�n actual: [ %1�, %2�, %3�, %4�, %5�, %6� ]")
//            .arg(angulos[0]).arg(angulos[1]).arg(angulos[2])
//            .arg(angulos[3]).arg(angulos[4]).arg(angulos[5]);
//
//        ui.lblPosicionActual->setText(texto);
//    }
//    else {
//        ui.lblPosicionActual->setText("Error al leer posici�n");
//    }
//}






using Mat4 = std::array<std::array<double, 4>, 4>;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

static Mat4 matIdentity() {
    Mat4 m{};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) m[i][j] = (i == j) ? 1 : 0;
    return m;
}
static Mat4 matMul(const Mat4& A, const Mat4& B) {
    Mat4 C = {};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
        C[i][j] = 0;
        for (int k = 0; k < 4; ++k) C[i][j] += A[i][k] * B[k][j];
    }
    return C;
}
static Mat4 Rz(double th) { double c = cos(th), s = sin(th); Mat4 m = matIdentity(); m[0][0] = c; m[0][1] = -s; m[1][0] = s; m[1][1] = c; return m; }
static Mat4 Ry(double th) { double c = cos(th), s = sin(th); Mat4 m = matIdentity(); m[0][0] = c; m[0][2] = s; m[2][0] = -s; m[2][2] = c; return m; }
static Mat4 Tz(double d) { Mat4 m = matIdentity(); m[2][3] = d; return m; }



void interfaz_robot::Directa() {

    // Leer �ngulos desde los spinbox
    double q1 = ui.spinEje0->value() * DEG2RAD;
    double q2 = ui.spinEje1->value() * DEG2RAD;
    double q3 = ui.spinEje2->value() * DEG2RAD;
    double q4 = ui.spinEje3->value() * DEG2RAD;
    double q5 = ui.spinEje4->value() * DEG2RAD;

    // Longitudes del robot (mm)
    const double a1 = 76, a2 = 125, a3 = 125, a4 = 60, a5 = 132;

    // Transformaci�n total
    Mat4 RTbt = matIdentity();
    RTbt = matMul(RTbt, Rz(q1));
    RTbt = matMul(RTbt, Tz(a1));
    RTbt = matMul(RTbt, Ry(q2));
    RTbt = matMul(RTbt, Tz(a2));
    RTbt = matMul(RTbt, Ry(q3));
    RTbt = matMul(RTbt, Tz(a3));
    RTbt = matMul(RTbt, Ry(q4));
    RTbt = matMul(RTbt, Tz(a4));
    RTbt = matMul(RTbt, Rz(q5));
    RTbt = matMul(RTbt, Tz(a5));

    // Posici�n
    double px = RTbt[0][3], py = RTbt[1][3], pz = RTbt[2][3];

    ui.lblPosicionActual->setText(
        QString("Pos: [X=%1, Y=%2, Z=%3] mm")
        .arg(px, 0, 'f', 1)
        .arg(py, 0, 'f', 1)
        .arg(pz, 0, 'f', 1)
    );

}





//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar�la�QLabel
//}

