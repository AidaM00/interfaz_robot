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
#include "camera_calibration.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
double q[6] = { 0,0,0,0,0,0 };   // Ángulos actuales del robot en grados

interfaz_robot::interfaz_robot(QWidget *parent)
    : QMainWindow(parent)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    ui.setupUi(this);
    camara = new CVideoAcquisition(0);
    // Conectar botones con sus slots
    connect(ui.btnInicio, SIGNAL(clicked()), this, SLOT(startStopCapture()));
    connect(ui.btnGuardar, SIGNAL(clicked()), this, SLOT(GuardarImagen()));
    connect(ui.btnMover1, SIGNAL(clicked()), this, SLOT(MoverEje()));
    connect(ui.btnMoverTodos, SIGNAL(clicked()), this, SLOT(MoverTodosLosEjes()));
    connect(ui.btnComunicacionrobot, SIGNAL(clicked()), this, SLOT(iniciarComRobot()));
    connect(ui.btnCalibrar, SIGNAL(clicked()), this, SLOT(CalibrarCamara()));
    connect(ui.spinFoco, SIGNAL(valueChanged(int)), camara, SLOT(setFoco(int)));
    connect(ui.btnCalibrarPanel, SIGNAL(clicked()), this, SLOT(CalibrarPanel()));
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
    ui.lblPosicionActual->setText("Posición actual: desconocida");
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
	ui.btnCalibrar->setEnabled(habilitar);
}

void interfaz_robot::startStopCapture()
{
    static bool capturando = false;
    capturando = !capturando;

    if (capturando) {
        camara->startStopCapture(true);  // iniciar cámara
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
        qDebug() << "Frame vacío recibido";
        return;
    }

    // Guardamos una copia del último frame mostrado
    ultimoFrame = frame.clone();

    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    QImage img((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);

    ui.lblInicio->setPixmap(QPixmap::fromImage(img));
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

void interfaz_robot::CalibrarCamara()
{
    //// Lista de archivos de calibración
    std::vector<std::string> archivos = {
    "calib_camara_01.jpg", "calib_camara_02.jpg", "calib_camara_03.jpg", "calib_camara_04.jpg", "calib_camara_05.jpg",
    "calib_camara_06.jpg", "calib_camara_07.jpg", "calib_camara_08.jpg", "calib_camara_09.jpg", "calib_camara_10.jpg",
    "calib_camara_11.jpg", "calib_camara_12.jpg", "calib_camara_13.jpg", "calib_camara_14.jpg", "calib_camara_15.jpg",
    "calib_camara_16.jpg", "calib_camara_17.jpg", "calib_camara_18.jpg", "calib_camara_19.jpg", "calib_camara_20.jpg"
    
    };

    calibrateCameraFromFiles(archivos);  // Llamada a la función
}

void interfaz_robot::CalibrarPanel()
{
    // Parámetros del panel
    cv::Size boardSize(9, 6);     // Esquinas internas
    float squareSize = 10.4f;     // mm

    // Cargar parámetros intrínsecos de la cámara
    cv::Mat K = leerMatriz("K.txt");
    cv::Mat D = leerMatriz("Kc.txt");
    if (K.empty() || D.empty()) 
    {
        QMessageBox::warning(this, "Error", "No se pudo leer K.txt o Kc.txt.");
        return;
    }

    // Nombre del archivo de la imagen del panel
    std::string imgFile = "calib_plano.jpg";

    // Nombre del archivo donde guardar la matriz RT
    std::string outFile = "RT_panel.txt";

    bool ok = calibratePanel(imgFile, K, D, boardSize, squareSize, outFile);

    if (ok)
    {
        QMessageBox::information(this, QString("Calibración panel"), QString("Matriz RT guardada en RT_panel.txt"));
    }
    else
    {
        QMessageBox::warning(this, QString("Error"), QString("La calibracion del panel fallo."));
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
        q[eje] = grados;
        m_robot->mover(eje, grados);
        QMessageBox::information(this, "Movimiento",
            QString("Eje %1 movido a %2 grados").arg(eje).arg(grados));
        Directa();  // Calcula y muestra la rotación y posición resultante
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

    for (int i = 0; i < 6; i++)
        q[i] = angulos[i];

    Directa();  // Calcula y muestra la rotación y posición resultante
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
//        QString texto = QString("Posición actual: [ %1°, %2°, %3°, %4°, %5°, %6° ]")
//            .arg(angulos[0]).arg(angulos[1]).arg(angulos[2])
//            .arg(angulos[3]).arg(angulos[4]).arg(angulos[5]);
//
//        ui.lblPosicionActual->setText(texto);
//    }
//    else {
//        ui.lblPosicionActual->setText("Error al leer posición");
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

    // Leer ángulos q1 a q5
    double q1 = q[0] * DEG2RAD;
    double q2 = q[1] * DEG2RAD;
    double q3 = q[2] * DEG2RAD;
    double q4 = q[3] * DEG2RAD;
    double q5 = q[4] * DEG2RAD;

    // Longitudes del robot (mm)
    const double a1 = 76, a2 = 125, a3 = 125, a4 = 60, a5 = 132;

    // Ángulos acumulados
    double ang3 = q2 + q3;
    double ang4 = q2 + q3 + q4;

    // ---- MATRIZ DE ROTACIÓN R ----
    // Preparación de senos y cosenos
    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2), s2 = sin(q2);
    double c5 = cos(q5), s5 = sin(q5);
    double sin_3 = sin(ang3), cos_3 = cos(ang3);
    double sin_4 = sin(ang4), cos_4 = cos(ang4);
    // Matriz R
    double R11 = c1 * cos_4 * c5 - s1 * s5;
    double R12 = -c1 * cos_4 * s5 - s1 * c5;
    double R13 = c1 * sin_4;
    double R21 = s1 * cos_4 * c5 + c1 * s5;
    double R22 = -s1 * cos_4 * s5 + c1 * c5;
    double R23 = s1 * sin_4;
    double R31 = -sin_4 * c5;
    double R32 = sin_4 * s5;
    double R33 = cos_4;
    // Mostrar la matriz R
    ui.lblRotacion->setText(
        QString(
            "Rot =\n"
            "[ %1  %2  %3 ]\n"
            "[ %4  %5  %6 ]\n"
            "[ %7  %8  %9 ]"
        ).arg(R11, 0, 'f', 3).arg(R12, 0, 'f', 3).arg(R13, 0, 'f', 3)
        .arg(R21, 0, 'f', 3).arg(R22, 0, 'f', 3).arg(R23, 0, 'f', 3)
        .arg(R31, 0, 'f', 3).arg(R32, 0, 'f', 3).arg(R33, 0, 'f', 3)
    );

	// ---- POSICIÓN P ----
    double S = a2 * s2 + a3 * sin_3 + (a4 + a5) * sin_4;
    double px = c1 * S;
    double py = s1 * S;
    double pz = a1 + a2 * c2 + a3 * cos_3 + (a4 + a5) * cos_4;
	// Mostrar la posición P
    //ui.lblPosicionActual->setText(
    //    QString("Pos: [X=%1, Y=%2, Z=%3] mm")
    //    .arg(px, 0, 'f', 1)
    //    .arg(py, 0, 'f', 1)
    //    .arg(pz, 0, 'f', 1)
    //);
    ui.lblPosicionActual->setText(
        QString("Pos actual: [ %1, %2, %3, %4, %5, %6 ] grados")
        .arg(q[0], 0, 'f', 1)
        .arg(q[1], 0, 'f', 1)
        .arg(q[2], 0, 'f', 1)
        .arg(q[3], 0, 'f', 1)
        .arg(q[4], 0, 'f', 1)
        .arg(q[5], 0, 'f', 1)
    );


    



    //// Transformación total
    //Mat4 RTbt = matIdentity();
    //RTbt = matMul(RTbt, Rz(q1));
    //RTbt = matMul(RTbt, Tz(a1));
    //RTbt = matMul(RTbt, Ry(q2));
    //RTbt = matMul(RTbt, Tz(a2));
    //RTbt = matMul(RTbt, Ry(q3));
    //RTbt = matMul(RTbt, Tz(a3));
    //RTbt = matMul(RTbt, Ry(q4));
    //RTbt = matMul(RTbt, Tz(a4));
    //RTbt = matMul(RTbt, Rz(q5));
    //RTbt = matMul(RTbt, Tz(a5));

    //// Posición
    //double px = RTbt[0][3], py = RTbt[1][3], pz = RTbt[2][3];

    //ui.lblPosicionActual->setText(
    //    QString("Pos: [X=%1, Y=%2, Z=%3] mm")
    //    .arg(px, 0, 'f', 1)
    //    .arg(py, 0, 'f', 1)
    //    .arg(pz, 0, 'f', 1)
    //);

}


void interfaz_robot::escribirMatriz(const std::string& nombreArchivo, const cv::Mat& M)
{
    std::ofstream archivo(nombreArchivo);
    if (!archivo.is_open()) { std::cerr << "No se pudo abrir " << nombreArchivo << "\n"; return; }

    archivo << M.rows << " " << M.cols << "\n";
    for (int i = 0; i < M.rows; ++i)
    {
        for (int j = 0; j < M.cols; ++j)
            archivo << M.at<double>(i, j) << " ";
        archivo << "\n";
    }
}

cv::Mat interfaz_robot::leerMatriz(const std::string& nombreArchivo)
{
    std::ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) { std::cerr << "No se pudo abrir " << nombreArchivo << "\n"; return cv::Mat(); }

    int filas, columnas;
    archivo >> filas >> columnas;
    cv::Mat M(filas, columnas, CV_64F);
    for (int i = 0; i < filas; ++i)
        for (int j = 0; j < columnas; ++j)
            archivo >> M.at<double>(i, j);
    return M;
}



//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar la QLabel
//}

