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
#include <iomanip>
#include <sstream>
#include <filesystem>
#include "procesado.h"
#include <filesystem>
#include <QEventLoop>

interfaz_robot::interfaz_robot(QWidget *parent)
    : QMainWindow(parent)
{
    //////////////////////////////////////////////////////////////
    double a[6];
    a[0] = 90;
    a[1] = 0;
    a[2] = 90;
    a[3] = 90;
    a[4] = 0;
    a[5] = 0;
    const double L1 = 76;
    const double L2 = 125;
    const double L3 = 125;
    const double L4 = 60;
    const double L5 = 132;

    cv:Mat m = fkBraccio(a);
    escribirMatriz("basura.txt",m);

    //////////////////////////////////////////////////////////////
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    ui.setupUi(this);

	//Cargar matrices de calibración
    CargarMatricesCalibracion();

    // Mostrar los valores iniciales de m_q[] en la interfaz
    ActualizarInterfaz();

    camara = new CVideoAcquisition(0);
    // Conectar botones con sus slots
    connect(ui.btnInicio, SIGNAL(toggled(bool)), this, SLOT(startStopCapture(bool)));
    connect(ui.btnGuardar, SIGNAL(clicked()), this, SLOT(GuardarImagen()));
    connect(ui.btnMover1, SIGNAL(clicked()), this, SLOT(MoverEje()));
    connect(ui.btnMoverTodos, SIGNAL(clicked()), this, SLOT(MoverTodosLosEjes()));
    connect(ui.btnComunicacionrobot, SIGNAL(clicked()), this, SLOT(iniciarComRobot()));
    connect(ui.btnCalibrar, SIGNAL(clicked()), this, SLOT(CalibrarCamara()));
    connect(ui.spinFoco, SIGNAL(valueChanged(int)), camara, SLOT(setFoco(int)));
    connect(ui.btnCalibrarPanel, SIGNAL(clicked()), this, SLOT(CalibrarPanel()));
    connect(ui.btnPoses, SIGNAL(clicked()), this, SLOT(GuardarImagenYPose()));
    connect(ui.btnCalibrarCamaraRobot, SIGNAL(clicked()), this, SLOT(CalibrarCamaraRobot()));
	connect(ui.btnPosInterm, SIGNAL(clicked()), this, SLOT(MoverPosInterm()));
    connect(ui.btnTrasladarPieza, SIGNAL(clicked()), this, SLOT(TrasladarPieza()));
    connect(ui.btnMoverPosicion, SIGNAL(clicked()), this, SLOT(MoverACota()));
    connect(ui.btnComenzar, SIGNAL(clicked()), this, SLOT(onComenzar()));
    connect(ui.btnCoger, SIGNAL(clicked()), this, SLOT(onCoger()));

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
    // Cámara
    ui.btnInicio->setEnabled(habilitar);
	ui.spinFoco->setEnabled(habilitar);
    ui.btnGuardar->setEnabled(habilitar);
    // Robot
    ui.btnCalibrar->setEnabled(habilitar);
    ui.btnCalibrarPanel->setEnabled(habilitar);
	ui.btnCalibrarCamaraRobot->setEnabled(habilitar);
	ui.btnComenzar->setEnabled(habilitar);
	ui.btnCoger->setEnabled(habilitar);
    ui.btnPoses->setEnabled(habilitar);
    ui.btnPosInterm->setEnabled(habilitar);
	ui.btnTrasladarPieza->setEnabled(habilitar);
    ui.btnMover1->setEnabled(habilitar);
    ui.btnMoverTodos->setEnabled(habilitar);
    ui.btnMoverPosicion->setEnabled(habilitar);
}

void interfaz_robot::startStopCapture(bool capturando)
{
    if (capturando) {
		m_capturando = true;
        getNewFrame();
        //camara->startStopCapture(true);  // Iniciar cámara
        //timerVideo->start(33);           // Actualizar cada 33 ms (unos 30 FPS)

        //ui.btnInicio->setText("Detener");
    }
    else {
        m_capturando = false;
        //timerVideo->stop();              // Detener refresco
        //camara->startStopCapture(false);
        //ui.btnInicio->setText("Iniciar");
    }
}


void interfaz_robot::getNewFrame()
{
    if (!m_capturando)
        return;

    // Si estamos mostrando frame congelado
    if (m_mostrarFrameCongelado)
    {
        // Mostrar SIEMPRE el mismo frame
        MostrarFrame(m_ultimoFrame, m_ultimosPuntos.centro, m_ultimosPuntos.lejano);
        QTimer::singleShot(30, this, SLOT(getNewFrame()));
        return;
    }

    // Modo vídeo normal
    MostrarVideo();

    if (m_comenzarProcesado)
    {
        cv::Mat frame = camara->getImage();

        PuntosProcesados pts = ComenzarProcesado(frame);

        m_ultimoFrame = frame.clone();
        m_ultimosPuntos = pts;
        std::cout << "Ult ptos: " << pts.centro << "y" << pts.lejano << std::endl;

        if (m_cogerPieza)
        {
            // Dibujar solo una vez
            cv::circle(m_ultimoFrame, cv::Point(m_ultimosPuntos.centro.x, m_ultimosPuntos.centro.y), 6, cv::Scalar(0, 0, 255), -1);
            cv::circle(m_ultimoFrame, cv::Point(m_ultimosPuntos.lejano.x, m_ultimosPuntos.lejano.y), 6, cv::Scalar(255, 0, 0), -1);

            cv::imwrite("pieza_final.png", m_ultimoFrame);

            // Centro del panel en coordenadas del robot (prueba)
            //cv::Point3d centro_baseRobot(170, 0, 0);
            //cv::Point3d lejano_baseRobot(172, 0, 1);
            //pts.centro = { 1019, 408 };
            //pts.lejano = { 1019, 444.469 };


            cv::Point3d centro_baseRobot =
                pixelToWorld3D(m_ultimosPuntos.centro, m_K, m_RTpc, m_RTcr);
            cv::Point3d lejano_baseRobot =
                pixelToWorld3D(m_ultimosPuntos.lejano, m_K, m_RTpc, m_RTcr);
            float angulo = atan2(
                lejano_baseRobot.x - centro_baseRobot.x,
                lejano_baseRobot.y - centro_baseRobot.y
            ) * 180.0 / M_PI;


            std::cout << "Centro en: " << centro_baseRobot << std::endl;
            std::cout << "Lejano en: " << lejano_baseRobot << std::endl;
            std::cout << "Angulo: " << angulo << std::endl;

            cv::Point2d pixel_prueba(896, 489);

            cv::Point3d punto_robot =
                pixelToWorld3D(pixel_prueba, m_K, m_RTpc, m_RTcr);

            std::cout << "Pixel (896, 489) -> Robot XYZ = "
                << punto_robot << std::endl;


                //CinematicaInversa(centro_baseRobot.x, centro_baseRobot.y, centro_baseRobot.z, angulo);
                //TrasladarPieza();
            std::array<int, 6> q_calculado =
                CinemInversa(centro_baseRobot.x, centro_baseRobot.y, centro_baseRobot.z, angulo);
            int q_traslado[6];
            for (int i = 0; i < 6; i++)
                q_traslado[i] = q_calculado[i];
            TrasladoPieza(q_traslado);

            // Pasar a modo frame congelado
            m_mostrarFrameCongelado = true;
            m_cogerPieza = false;
            m_comenzarProcesado = false;
        }
    }

    QTimer::singleShot(30, this, SLOT(getNewFrame()));
}



void interfaz_robot::MostrarVideo()
{
    camara->startStopCapture(true);
    cv::Mat frame = camara->getImage();
    if (frame.empty()) {
        qDebug() << "Frame vacío recibido";
        return;
    }

    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);
    QImage img((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);

    ui.lblInicio->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui.lblInicio->setPixmap(
        QPixmap::fromImage(img).scaled(
            ui.lblInicio->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
}

void interfaz_robot::MostrarFrame(cv::Mat frame, Point2f centro, Point2f lejano)
{
    if (frame.empty()) {
        qDebug() << "Frame vacío recibido";
        return;
    }

    cv::Mat rgbFrame;
    cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);

    QImage img((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);

    ui.lblInicio->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui.lblInicio->setPixmap(
        QPixmap::fromImage(img).scaled(
            ui.lblInicio->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
}



void interfaz_robot::onComenzar()
{
    m_comenzarProcesado = true;
    m_mostrarFrameCongelado = false;
}

void interfaz_robot::onCoger()
{
    m_cogerPieza = true;
}

void interfaz_robot::CargarMatricesCalibracion()
{
    m_K = leerMatriz("K.txt");
    m_D = leerMatriz("Kc.txt");
    m_RTpc = leerMatriz("RT_panel_camara.txt");
    m_RTcr = leerMatriz("RT_camara_base.txt");

    if (m_K.empty() || m_D.empty()) {
        QMessageBox::critical(this, "Error",
            "No se pudieron cargar las matrices intrínsecas de la cámara.");
        return;
    }

    if (m_RTpc.empty()) {
        qDebug() << "Aviso: RT_panel_camara no cargada.";
    }

    if (m_RTcr.empty()) {
        qDebug() << "Aviso: RT_camara_base no cargada.";
    }

    qDebug() << "Matrices de calibracion cargadas correctamente.";
}


void interfaz_robot::GuardarImagen()
{
    cv::Mat img = camara->getImage();  // Obtener la imagen actual de la cámara
    if (!img.empty()) {
        // Obtener la fecha y hora actual
        std::time_t t = std::time(nullptr);
        std::tm now;
        localtime_s(&now, &t);  // Usar la versión segura localtime_s

        // Crear nombre de archivo con timestamp: captura_YYYYMMDD_HHMMSS.png
        char nombreArchivo[50];
        std::strftime(nombreArchivo, sizeof(nombreArchivo), "captura_%Y%m%d_%H%M%S.png", &now);

        cv::imwrite(nombreArchivo, img);  // Guardar la imagen en un archivo
    }
    else {
        qDebug() << "Error: No hay imagen para guardar.";
    }
}

void interfaz_robot::CalibrarCamara()
{
    // Lista de archivos de calibración
    std::vector<std::string> archivos = {
    "calibr_camara_01.png", "calibr_camara_02.png", "calibr_camara_03.png", "calibr_camara_04.png", "calibr_camara_05.png",
    "calibr_camara_06.png", "calibr_camara_07.png", "calibr_camara_08.png", "calibr_camara_09.png", "calibr_camara_10.png",
    "calibr_camara_11.png", "calibr_camara_12.png", "calibr_camara_13.png", "calibr_camara_14.png", "calibr_camara_15.png",
    "calibr_camara_16.png", "calibr_camara_17.png", "calibr_camara_18.png", "calibr_camara_19.png"
    };

    calibrateCameraFromFiles(archivos);  // Llamada a la función
}

void interfaz_robot::CalibrarPanel()
{
    if (m_K.empty() || m_D.empty()) 
    {
        QMessageBox::warning(this, "Error", "No se pudo leer K.txt o Kc.txt.");
        return;
    }

    // Nombre del archivo de la imagen del panel
    std::string imgFile = "calib_panel.png";

    // Nombre del archivo donde guardar la matriz RT
    std::string outFile = "RT_panel_camara.txt";

    bool ok = calibratePanel(imgFile, m_K, m_D, boardSize, m_squareSize, outFile);

    if (ok)
    {
        QMessageBox::information(this, QString("Calibración panel"), QString("Matriz RT guardada en RT_panel_camara.txt"));
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

	MoverEje(eje, grados);
}

void interfaz_robot::MoverEje(int indexEje, int grados)
{
    // Validación
    if (grados < -90 || grados > 90) {
        QMessageBox::warning(this, "Error", "El valor de grados debe estar entre -90 y 90.");
        return;
    }

    // Enviar comando al robot
    if (m_robot) {
        qDebug() << "Moviendo eje" << indexEje << "a" << grados << "grados.";
		m_q[indexEje] = grados; //Actualizar ángulo actual
        m_robot->mover(indexEje, grados);
        /*QMessageBox::information(this, "Movimiento",
            QString("Eje %1 movido a %2 grados").arg(indexEje).arg(grados));*/

        // Actualizar información de los motores en la interfaz
        ActualizarInterfaz();
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
	MoverTodosLosEjes(angulos);
    
}

void interfaz_robot::MoverTodosLosEjes(int *angulos)
{
    //// Verificar que todos los valores estén dentro del rango permitido
    //for (int i = 0; i < 6; ++i)
    //{
    //    if (angulos[i] < -90 || angulos[i] > 90)
    //    {
    //        QMessageBox::warning(this, "Valor fuera de rango",
    //            QString("El eje %1 tiene un valor no válido (%2°).\n\n"
    //                "Debe estar entre -90° y +90°.")
    //            .arg(i).arg(angulos[i]));
    //        return; // Cancelar el envío
    //    }
    //}

    // Construir el comando tipo: #a45,0,30,90,-30,60*
    QString comando = "#a";
    for (int i = 0; i < 6; ++i) {
        comando += QString::number(angulos[i]);
        if (i < 5) comando += ",";
    }
    comando += "*";

    // Enviar el comando al robot
    if (m_robot)
        m_robot->enviarComando(comando);
    qDebug() << "Comando enviado al robot:" << comando;
   /* QMessageBox::information(this, "Movimiento",
        "Todos los ejes se están moviendo simultáneamente a las nuevas posiciones.");*/

	// Actualizar los ángulos actuales
    for (int i = 0; i < 6; i++)
        m_q[i] = angulos[i];

    // Actualizar información de los motores en la interfaz
    ActualizarInterfaz();
}

void interfaz_robot::MoverPosInterm()
{
    if (!m_robot) {
        QMessageBox::warning(this, "Error", "Robot no inicializado.");
        return;
    }

    // Valores deseados para cada eje: 45, 0, 70, 70, 0, 0
    double ang_deseado[6] = { 45, 0, 70, 70, 0, 0 };

    // Construir comando tipo: #a45,0,70,70,0,0*
    QString comando = "#a";
    for (int i = 0; i < 6; ++i) {
        comando += QString::number(ang_deseado[i]);
        if (i < 5) comando += ",";
    }
    comando += "*";

    // Enviar comando al robot
    m_robot->enviarComando(comando);
    qDebug() << "Moviendo robot a posición intermedia: " << comando;

    // Actualizar los ángulos actuales
    for (int i = 0; i < 6; i++)
        m_q[i] = ang_deseado[i];

    // Actualizar información de los motores en la interfaz
    ActualizarInterfaz();
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


void interfaz_robot::ActualizarInterfaz() {
    QString texto = QString("Motores: %1, %2, %3, %4, %5, %6")
        .arg(m_q[0])
        .arg(m_q[1])
        .arg(m_q[2])
        .arg(m_q[3])
        .arg(m_q[4])
        .arg(m_q[5]);

    ui.lblMotores->setText(texto);
}

void interfaz_robot::escribirMatriz(const std::string& nombreArchivo, const cv::Mat& M)
{
    std::ofstream archivo(nombreArchivo);
    if (!archivo.is_open()) { std::cerr << "No se pudo abrir " << nombreArchivo << "\n"; return; }

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
    if (!archivo.is_open()) {
        std::cerr << "No se pudo abrir " << nombreArchivo << "\n";
        return cv::Mat();
    }

    std::vector<std::vector<double>> valores;
    std::string linea;

    // Leer cada línea del archivo
    while (std::getline(archivo, linea)) {
        if (linea.empty()) continue; // Saltar líneas vacías
        std::stringstream ss(linea);
        double val;
        std::vector<double> fila;
        while (ss >> val)
            fila.push_back(val);
        valores.push_back(fila);
    }

    if (valores.empty()) return cv::Mat();

    int filas = static_cast<int>(valores.size());
    int columnas = static_cast<int>(valores[0].size());

    // Crear la matriz final
    cv::Mat M(filas, columnas, CV_64F);
    for (int i = 0; i < filas; ++i)
        for (int j = 0; j < columnas; ++j)
            M.at<double>(i, j) = valores[i][j];

    return M;
}

// Función para guardar imagen y pose del robot
void interfaz_robot::GuardarImagenYPose()
{
    cv::Mat img = camara->getImage();  // Obtener la imagen actual de la cámara
    if (img.empty()) {
        qDebug() << "Error: no hay imagen para guardar.";
        return;
    }

    // Leer valores actuales de los spinbox
    int q_spin[6] = {
        ui.spinEje0->value(),
        ui.spinEje1->value(),
        ui.spinEje2->value(),
        ui.spinEje3->value(),
        ui.spinEje4->value(),
        ui.spinEje5->value()
    };

    // Usamos el mismo contador para imagen y pose
    int numArchivo = contador;

    // Guardar la imagen
    std::ostringstream nombreImagen;
    nombreImagen << "imagen_" << std::setw(2) << std::setfill('0') << numArchivo << ".png";
    cv::imwrite(nombreImagen.str(), img);
    qDebug() << "Imagen guardada:" << QString::fromStdString(nombreImagen.str());

    // Guardar la pose
    std::ostringstream nombrePose;
    nombrePose << "pose_" << std::setw(2) << std::setfill('0') << numArchivo << ".txt";

    std::ofstream archivoPose(nombrePose.str());
    if (archivoPose.is_open()) {
        for (int i = 0; i < 6; ++i)
            archivoPose << q_spin[i] << (i < 5 ? " " : "");
        archivoPose << std::endl;
        archivoPose.close();
        qDebug() << "Pose guardada:" << QString::fromStdString(nombrePose.str());

        // Actualizar los ángulos actuales
        for (int i = 0; i < 6; i++)
            m_q[i] = q_spin[i];

        // Actualizar información de los motores en la interfaz
        ActualizarInterfaz();
    }
    else {
        qDebug() << "Error: no se pudo abrir el archivo de pose.";
    }

    // Incrementar el contador una sola vez
    contador++;
}

void interfaz_robot::CalibrarCamaraRobot()
{
    // Contar cuántas parejas imagen-pose existen
    int numImages = 0;
    for (const auto& entry : fs::directory_iterator(".")) {
        std::string name = entry.path().filename().string();
        if (name.find("imagen_") == 0 && name.find(".png") != std::string::npos) {
            numImages++;
        }
    }

    if (numImages == 0) {
        QMessageBox::warning(this, "Error", "No se encontraron imágenes para calibrar.");
        return;
    }

    if (m_K.empty() || m_D.empty()) {
        qDebug() << "Error: No se pudo leer K.txt o Kc.txt.";
        return;
    }

    // Archivo de salida
    string outFile = "RT_camara_base.txt";

    // Ejecutar calibración
    bool ok = calibrateCameraRobot(numImages, boardSize, m_squareSize, m_K, m_D, outFile);

    if (ok)
        qDebug() << "Calibracion completa. RT camara-robot guardada";
    else
        qDebug() << "Error: La calibracion camara-robot fallo.";
}

//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar la QLabel
//}


PuntosProcesados interfaz_robot::ComenzarProcesado(Mat img)
{
    // Recortar y reescalar
    Mat procesada = recortarYReescalar(img);

    // Localizar pieza: centro y lejano
    cv::Point2f centro(0, 0);
    cv::Point2f lejano(0, 0);
    LocalizarPieza(procesada, centro, lejano);
    std::cout << "Centroide: " << centro << ", Punto lejano: " << lejano << std::endl;

    // Quitar reescalado
    // Definir los mismos puntos que se usaron en recortarYReescalar
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(468, 252),    // Superior izquierda
        cv::Point2f(1318, 267),   // Superior derecha
        cv::Point2f(271, 1048),    // Inferior izquierda
        cv::Point2f(1487, 1044)   // Inferior derecha
    };

    int anchoNuevo = 1920; // Igual que en recortarYReescalar
    int altoNuevo = 1080;

    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(anchoNuevo, 0),
        cv::Point2f(0, altoNuevo),
        cv::Point2f(anchoNuevo, altoNuevo)
    };

    // Calcular homografía inversa
    cv::Mat hInv = cv::getPerspectiveTransform(dstPoints, srcPoints);

    // Transformar los puntos detectados a coordenadas originales
    std::vector<cv::Point2f> puntosProcesados = { centro, lejano };
    cv::perspectiveTransform(puntosProcesados, puntosProcesados, hInv);

    // Actualizar los puntos con las coordenadas originales
    centro = puntosProcesados[0];
    lejano = puntosProcesados[1];

    std::cout << "Centroide transf: " << centro << ", Punto lejano transf: " << lejano << std::endl;


    // Quitar distorsión
    std::vector<cv::Point2d> srcPoints1{ centro }, undistortedPoints_centro;
    cv::undistortPoints(srcPoints1, undistortedPoints_centro, m_K, m_D, cv::noArray(), m_K);
    cv::Point2d centro_sin_distorsion = undistortedPoints_centro[0];

    std::vector<cv::Point2d> srcPoints2{ lejano }, undistortedPoints_lejano;
    cv::undistortPoints(srcPoints2, undistortedPoints_lejano, m_K, m_D, cv::noArray(), m_K);
    cv::Point2d lejano_sin_distorsion = undistortedPoints_lejano[0];

    std::cout << "Centro sin dist" << centro_sin_distorsion << ". Lejano sin disr: " << lejano_sin_distorsion << std::endl;

    return { centro_sin_distorsion, lejano_sin_distorsion };
    //return { centro, lejano };
}
cv::Point3d interfaz_robot::pixelToWorld3D(const cv::Point2d& uv, // Esto uv ya tiene que ser sin distorsión
    const cv::Mat& K,
    const cv::Mat& RTpanelCam,
    const cv::Mat& RTcamRobot)
{
    // Extraer fx, fy, cx, cy
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    // Coordenadas normalizadas de la cámara (recta de visión)
    double x = (uv.x - cx) / fx;
    double y = (uv.y - cy) / fy;
    cv::Mat Pcam = (cv::Mat_<double>(4, 1) << x, y, 1.0, 1.0);

    // Plano definido por RTpanel-camara
    // P0 y P1 en coordenadas de cámara
    cv::Mat P0c = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1); // Origen del plano
    cv::Mat P1c = (cv::Mat_<double>(4, 1) << 0, 0, 1, 1); // Punto a 1 unidad en z

    // Transformar puntos al sistema de la cámara
    P0c = RTpanelCam * P0c;
    P1c = RTpanelCam * P1c;

    // Vector normal del plano
    cv::Mat Vn = P1c - P0c;
    double A = Vn.at<double>(0, 0);
    double B = Vn.at<double>(1, 0);
    double C = Vn.at<double>(2, 0);
    double D = -(A * P0c.at<double>(0, 0) + B * P0c.at<double>(1, 0) + C * P0c.at<double>(2, 0));

    // Intersección de la recta con el plano: X = t * (x,y,1)
    double t = -(D) / (A * x + B * y + C * 1.0);
    double Ix = t * x;
    double Iy = t * y;
    double Iz = t * 1.0;

    // Transformar al sistema del robot
    cv::Matx44d RT(RTcamRobot); // 4x4
    cv::Vec4d P(Ix, Iy, Iz, 1.0);
    cv::Vec4d P_robot = RT * P;

    // Extraer punto, tener en cuenta problemas de holguras
    double Xr = 319-P_robot[0];
    double Yr = P_robot[1]+20;// aplicar una cierta correccion por las holguras para que vaya mas al centro de la pieza
    double Zr = P_robot[2];

    return cv::Point3d(Xr, Yr, Zr);
}



void interfaz_robot::AbrirCerrarPinza(int accion)    // 0 = abrir, 1 = cerrar
{
    if (!m_robot) {
        QMessageBox::warning(this, "Error", "Robot no inicializado.");
        return;
    }

    if (accion == 0) {
        m_q[5] = 0;   // Abrir
    }
    else if (accion == 1) {
        m_q[5] = 85;  // Cerrar
    }
    else {
        QMessageBox::warning(this, "Error", "Acción inválida. Usa 0 para abrir, 1 para cerrar.");
        return;
    }

    // Construir comando para el robot
    QString comando = QString("#m5,%1*").arg(m_q[5]);

    // Enviar el comando
    m_robot->enviarComando(comando);

    qDebug() << "Pinza movida. Comando enviado:" << comando;

    // Actualizar información de los motores en la interfaz
    ActualizarInterfaz();
}

//  CINEMÁTICA INVERSA 
std::array<int, 6> interfaz_robot::CinemInversa(
    double cx, double cy, double cz, double angulo)
{
    std::array<int, 6> q_resultado = { 0, 0, 0, 0, 0, 0 };

    double z = cz;
    double r = sqrt(cx * cx + cy * cy);
    double X = z - a1 + a4 + a5;

    // q3
    double num_q3 = X * X + r * r - a2 * a2 - a3 * a3;
    double den_q3 = 2 * a2 * a3;
    double arg_q3 = num_q3 / den_q3;
    arg_q3 = std::clamp(arg_q3, -1.0, 1.0);
    double q3_rad = acos(arg_q3);

    // q2
    double A = a2 + a3 * cos(q3_rad);
    double B = a3 * sin(q3_rad);
    double num_q2 = X * A + B * r;
    double den_q2 = A * A + B * B;
    double arg_q2 = num_q2 / den_q2;
    arg_q2 = std::clamp(arg_q2, -1.0, 1.0);
    double q2_rad = acos(arg_q2);

    // q4 (muñeca)
    double q4_rad = M_PI - (q2_rad + q3_rad);

    // Convertir a grados
    double q_deseado[6];
    q_deseado[0] = atan2(cy, cx) * RAD2DEG;
    q_deseado[1] = q2_rad * RAD2DEG;
    q_deseado[2] = q3_rad * RAD2DEG;
    q_deseado[3] = q4_rad * RAD2DEG;
    q_deseado[4] = angulo - q_deseado[0];
    std::cout << "Angulo q_deseado[0]: " << q_deseado[0] << std::endl;
    std::cout << "Angulo q_deseado[1]: " << q_deseado[1] << std::endl;
    std::cout << "Angulo q_deseado[2]: " << q_deseado[2] << std::endl;
    std::cout << "Angulo q_deseado[3]: " << q_deseado[3] << std::endl;
    std::cout << "Angulo q_deseado[4]: " << q_deseado[4] << std::endl;
    q_deseado[5] = 0;

    for (int i = 0; i < 6; i++)
        q_resultado[i] = static_cast<int>(std::floor(q_deseado[i]));

    return q_resultado;
}

void interfaz_robot::TrasladoPieza(int q_destino[6])
{
    // Comprobación de límites
    m_posicion_valida = true;
    QString mensaje_error;

    for (int i = 0; i < 6; i++)
    {
        int min_ang = 0;
        int max_ang = 90;

        if (i == 0) min_ang = -30;
        if (i == 4) min_ang = -180;

        if (q_destino[i] < min_ang || q_destino[i] > max_ang)
        {
            m_posicion_valida = false;
            mensaje_error += QString(
                "Eje %1 fuera de rango: %2° (rango %3°–%4°)\n"
            ).arg(i + 1).arg(q_destino[i]).arg(min_ang).arg(max_ang);
        }
    }

    if (!m_posicion_valida)
    {
        QMessageBox::warning(
            this,
            "Posición no alcanzable",
            mensaje_error
        );
        return;
    }

    // 1️ Posición de seguridad inicial (pinza abierta)
    MoverEje(3, 90);
    waitKey(500);


    // 2️ Ir a la posición objetivo (IK) con pinza abierta
    qDebug() << "Moviendo a posición objetivo";
    MoverTodosLosEjes(q_destino);
    waitKey(5000);
    // Actualizar estado interno
    for (int i = 0; i < 6; i++)
        m_q[i] = q_destino[i];
    ActualizarInterfaz();

    // 3️ Cerrar pinza
    AbrirCerrarPinza(1);
    waitKey(1200);
    ActualizarInterfaz();

    // 5️ Ir a posición de depósito
    int depositar[6] = { 90, 0, 70, 90, 0, m_q[5] };
    MoverTodosLosEjes(depositar);
    waitKey(5000);
    // Actualizar estado
    for (int i = 0; i < 6; i++)
        m_q[i] = depositar[i];
    ActualizarInterfaz();
    MoverEje(1, 10);
    waitKey(5000);
    ActualizarInterfaz();

    // 6️ Soltar pieza
    AbrirCerrarPinza(0);
    waitKey(5000);
    ActualizarInterfaz();

    // 7️ Volver a posición inicial
    int inicial[6] = { 0, 0, 0, 0, 0, 0 };
    MoverTodosLosEjes(inicial);
    waitKey(2000);
    for (int i = 0; i < 6; i++)
        m_q[i] = inicial[i];
    ActualizarInterfaz();

    qDebug() << "Pieza depositada";
}

void interfaz_robot::MoverACota()
{
    double punto3D[3] = {
        ui.spinX->value(),
        ui.spinY->value(),
        ui.spinZ->value()
    };

    double angulo = ui.spinA->value();
    std::array<int, 6> q_calculado =
        CinemInversa(punto3D[0], punto3D[1], punto3D[2], angulo);
	int q_traslado[6];
    for (int i = 0; i < 6; i++)
        q_traslado[i] = q_calculado[i];
    TrasladoPieza(q_traslado);
}


