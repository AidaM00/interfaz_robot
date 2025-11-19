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
    connect(ui.btnPoses, SIGNAL(clicked()), this, SLOT(GuardarImagenYPose()));
    connect(ui.btnCalibrarCamaraRobot, SIGNAL(clicked()), this, SLOT(CalibrarCamaraRobot()));
    connect(ui.btnProcesar, SIGNAL(clicked()), this, SLOT(ProcesarImagen()));
	connect(ui.btnPosInterm, SIGNAL(clicked()), this, SLOT(MoverPosInterm()));
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

    ui.lblInicio->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui.lblInicio->setPixmap(
        QPixmap::fromImage(img).scaled(
            ui.lblInicio->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
    

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
    "calibr_camara_16.png", "calibr_camara_17.png", "calibr_camara_18.png", "calibr_camara_19.png", "calibr_camara_20.png"
    };

    calibrateCameraFromFiles(archivos);  // Llamada a la función
}

void interfaz_robot::CalibrarPanel()
{
    // Cargar parámetros intrínsecos de la cámara
    cv::Mat K = leerMatriz("K.txt");
    cv::Mat D = leerMatriz("Kc.txt");
    if (K.empty() || D.empty()) 
    {
        QMessageBox::warning(this, "Error", "No se pudo leer K.txt o Kc.txt.");
        return;
    }

    // Nombre del archivo de la imagen del panel
    std::string imgFile = "calib_panel.png";

    // Nombre del archivo donde guardar la matriz RT
    std::string outFile = "RT_panel_camara.txt";

    bool ok = calibratePanel(imgFile, K, D, boardSize, m_squareSize, outFile);

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
    QMessageBox::information(this, "Movimiento",
        "Todos los ejes se están moviendo simultáneamente a las nuevas posiciones.");

    for (int i = 0; i < 6; i++)
        q[i] = angulos[i];

    Directa();  // Calcula y muestra la rotación y posición resultante
}

void interfaz_robot::MoverPosInterm()
{
    if (!m_robot) {
        QMessageBox::warning(this, "Error", "Robot no inicializado.");
        return;
    }

    // Valores deseados para cada eje: 45, 0, 70, 70, 0, 0
    q[0] = 45;
    q[1] = 0;
    q[2] = 70;
    q[3] = 70;
    q[4] = 0;
    q[5] = 0;

    // Construir comando tipo: #a45,0,70,70,0,0*
    QString comando = "#a";
    for (int i = 0; i < 6; ++i) {
        comando += QString::number(q[i]);
        if (i < 5) comando += ",";
    }
    comando += "*";

    // Enviar comando al robot
    m_robot->enviarComando(comando);
    qDebug() << "Moviendo robot a posición intermedia: " << comando;

    // Actualizar la interfaz con la posición
    Directa();
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
    // Convertir ángulos a radianes
    double q1 = q[0] * DEG2RAD;
    double q2 = q[1] * DEG2RAD;
    double q3 = q[2] * DEG2RAD;
    double q4 = q[3] * DEG2RAD;
    double q5 = q[4] * DEG2RAD;

    // Ángulos acumulados
    double ang3 = q2 + q3;
    double ang4 = q2 + q3 + q4;

    // Preparación de senos y cosenos
    double c1 = cos(q1), s1 = sin(q1);
    double c2 = cos(q2), s2 = sin(q2);
    double c5 = cos(q5), s5 = sin(q5);
    double sin_3 = sin(ang3), cos_3 = cos(ang3);
    double sin_4 = sin(ang4), cos_4 = cos(ang4);

    // Matriz de rotación R
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

    // Posición P
    double S = a2 * s2 + a3 * sin_3 + (a4 + a5) * sin_4;
    double px = c1 * S;
    double py = s1 * S;
    double pz = a1 + a2 * c2 + a3 * cos_3 + (a4 + a5) * cos_4;

    // Mostrar la posición
    ui.lblPosicionActual->setText(
        QString("Pos actual: [ %1, %2, %3, %4, %5, %6 ] grados")
        .arg(q[0], 0, 'f', 1)
        .arg(q[1], 0, 'f', 1)
        .arg(q[2], 0, 'f', 1)
        .arg(q[3], 0, 'f', 1)
        .arg(q[4], 0, 'f', 1)
        .arg(q[5], 0, 'f', 1)
    );
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
        if (linea.empty()) continue; // saltar líneas vacías
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
            archivoPose << q[i] << (i < 5 ? " " : "");
        archivoPose << std::endl;
        archivoPose.close();
        qDebug() << "Pose guardada:" << QString::fromStdString(nombrePose.str());
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

    // Leer parámetros de cámara ya calibrada
    Mat K = leerMatriz("K.txt");
    Mat D = leerMatriz("Kc.txt");
    if (K.empty() || D.empty()) {
        qDebug() << "Error: No se pudo leer K.txt o Kc.txt.";
        return;
    }

    // Archivo de salida
    string outFile = "RT_camara_robot.txt";

    // Ejecutar calibración
    bool ok = calibrateCameraRobot(numImages, boardSize, m_squareSize, K, D, outFile);

    if (ok)
        qDebug() << "Calibracion completa. RT camara-robot guardada";
    else
        qDebug() << "Error: La calibracion camara-robot fallo.";
}

//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar la QLabel
//}



void interfaz_robot::ProcesarImagen()
{
    namespace fs = std::filesystem;
    fs::path rutaEjecutable = fs::current_path();
    std::vector<fs::path> imagenes;

    // Buscar imágenes "pieza_*.png"
    for (const auto& entry : fs::directory_iterator(rutaEjecutable)) {
        std::string nombre = entry.path().filename().string();
        if (nombre.rfind("pieza_", 0) == 0 &&
            nombre.find("segmentada") == std::string::npos &&
            entry.path().extension() == ".png") {
            imagenes.push_back(entry.path());
        }
    }

    if (imagenes.empty()) {
        std::cout << "No se encontraron imágenes que empiecen por 'pieza_'.\n";
        return;
    }

    std::sort(imagenes.begin(), imagenes.end());

    for (const auto& rutaImagen : imagenes) {
        std::cout << "\nProcesando: " << rutaImagen << std::endl;
        cv::Mat img = cv::imread(rutaImagen.string());
        if (img.empty()) {
            std::cerr << "No se pudo cargar " << rutaImagen << "\n";
            continue;
        }

        // Recortar y reescalar
        cv::Mat procesada = recortarYReescalar(img);
        if (procesada.empty()) {
            std::cerr << "Error: resultado vacío tras recortarYReescalar.\n";
            continue;
        }

        // Localizar pieza y guardar imagen 
        cv::Point2f centro(0, 0);
        cv::Point2f lejano(0,0);
        LocalizarPieza(procesada, centro, lejano);
        std::cout << "Centroide: " << centro << ", Punto lejano: " << lejano << std::endl;

        Mat K = leerMatriz("K.txt");
        Mat RTpc = leerMatriz("RT_panel_camara.txt");
        Mat RTcr = leerMatriz("RT_camara_robot.txt");
        cv::Mat distCoeffs = interfaz_robot::leerMatriz("Kc.txt");
        // Quitar distorsión
        std::vector<cv::Point2d> srcPoints1{ centro }, undistortedPoints_centro;
        cv::undistortPoints(srcPoints1, undistortedPoints_centro, K, distCoeffs, cv::noArray(), K);
        cv::Point2d centro_sin_distorsion = undistortedPoints_centro[0];

        std::vector<cv::Point2d> srcPoints2{ lejano }, undistortedPoints_lejano;
        cv::undistortPoints(srcPoints2, undistortedPoints_lejano, K, distCoeffs, cv::noArray(), K);
        cv::Point2d lejano_sin_distorsion = undistortedPoints_lejano[0];

        cv::Point3d centro_baseRobot = pixelToWorld3D(centro_sin_distorsion, K, RTpc, RTcr); //Mat& RTpanelCam, Mat& RTcamRobot
        cv::Point3d lejano_baseRobot = pixelToWorld3D(lejano_sin_distorsion, K, RTpc, RTcr);

		float angulo = atan2(lejano_baseRobot.y - centro_baseRobot.y,
			lejano_baseRobot.x - centro_baseRobot.x) * RAD2DEG;

		CinematicaInversa(centro_baseRobot.x, centro_baseRobot.y, centro_baseRobot.z, angulo);

        // Mover agarrin
    }
}




//cv::Mat interfaz_robot::ProcesarImagen() {
//    if (!camara) {
//        return cv::Mat();
//    }
//
//    std::cout << "Procesamiento en tiempo real iniciado\n";
//
//    cv::Mat frame, procesada, salida;
//
//    while (true) {
//        frame = camara->getImage();
//        if (frame.empty()) {
//            std::cerr << "Frame vacío recibido.\n";
//            continue;
//        }
//
//        // Recortar y reescalar el frame actual
//        procesada = recortarYReescalar(frame);
//        cv::imshow("Procesada", procesada);
//        if (procesada.empty()) {
//            std::cerr << "Error al recortar y reescalar.\n";
//            continue;
//        }
//
//        // Procesar la imagen en tiempo real
//        salida = Segmentacion(procesada);
//
//        // Mostrar el resultado segmentado (con centroide y ángulo dibujados)
//        cv::imshow("Vista segmentada", salida);
//
//        // Pequeña espera para refrescar ventana (33 ms, 30 FPS)
//        char c = static_cast<char>(cv::waitKey(33));
//        if (c == 'q' || c == 27)
//            break;
//    }
//
//    cv::destroyAllWindows();
//    return salida;
//}

cv::Point3d interfaz_robot::pixelToWorld3D(const cv::Point2d& uv, // esto uv ya tiene que ser sin distorsión
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
    cv::Mat P0c = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);  // origen del plano
    cv::Mat P1c = (cv::Mat_<double>(4, 1) << 0, 0, 1, 1);  // punto a 1 unidad en z

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
    cv::Mat Ptx = (cv::Mat_<double>(4, 1) << Ix, Iy, Iz, 1);
    cv::Mat Probot = RTcamRobot * Ptx;

    double Xr = Probot.at<double>(0, 0);
    double Yr = Probot.at<double>(1, 0);
    double Zr = Probot.at<double>(2, 0);

    return cv::Point3d(Xr, Yr, Zr);
    
}



void interfaz_robot::MoverRobotActual()
{
    if (!m_robot) return;

    // Construir comando con los q actuales
    QString comando = "#a";
    for (int i = 0; i < 6; ++i) {
        comando += QString::number(q[i]);
        if (i < 5) comando += ",";
    }
    comando += "*";

    // Enviar al robot
    m_robot->enviarComando(comando);

    qDebug() << "Robot moviéndose a: " << comando;
    Directa(); // Actualizar posición en la interfaz
}

void interfaz_robot::AbrirCerrarPinza(int accion) //0 = abrir, 1 = cerrar
{
    if (!m_robot) {
        QMessageBox::warning(this, "Error", "Robot no inicializado.");
        return;
    }

    if (accion == 0) {
        q[5] = 0;   // Abrir
    }
    else if (accion == 1) {
        q[5] = 85;  // Cerrar
    }
    else {
        QMessageBox::warning(this, "Error", "Acción inválida. Usa 0 para abrir, 1 para cerrar.");
        return;
    }

    // Construir comando para el robot
    QString comando = QString("#m5,%1*").arg(q[5]);

    // Enviar el comando
    m_robot->enviarComando(comando);

    qDebug() << "Pinza movida. Comando enviado:" << comando;
    Directa(); // Actualizar la interfaz
}


void interfaz_robot::CinematicaInversa(double cx, double cy, double cz, double angulo)
{
    double z = cz;
	double r = sqrt(pow(cx, 2) + pow(cy, 2));
    // Variable auxiliar X 
    double X = z - a1 + a4 + a5;

    // Cálculo de q3 
    double num_q3 = pow(X, 2) + pow(r, 2) - pow(a2, 2) - pow(a3, 2); //pow(base, exponente)
    double den_q3 = 2 * a2 * a3;
    double arg_q3 = num_q3 / den_q3;
    // Limitar el dominio de acos
    if (arg_q3 > 1.0) arg_q3 = 1.0;
    if (arg_q3 < -1.0) arg_q3 = -1.0;
    double q3_rad = acos(arg_q3);

    // Cálculo de q2
    double A = a2 + a3 * cos(q3_rad);
    double B = a3 * sin(q3_rad);
    double num_q2 = X * A + B * r;
    double den_q2 = pow(A, 2) + pow(B, 2);
    double arg_q2 = num_q2 / den_q2;
    if (arg_q2 > 1.0) arg_q2 = 1.0;
    if (arg_q2 < -1.0) arg_q2 = -1.0;
    double q2_rad = acos(arg_q2);

    // Cálculo de q4 según q2 + q3 + q4 = 180º 
    double q4_rad = M_PI - (q2_rad + q3_rad);

    // Guardar resultados en q[6] (grados)
	q[0] = atan2(cy,cx) * RAD2DEG; // q1 en grados
    q[1] = q2_rad * RAD2DEG;
    q[2] = q3_rad * RAD2DEG;
    q[3] = q4_rad * RAD2DEG;
	q[4] = angulo; 
	q[5] = 0; // Pinza abierta

    // Mostrar valores
    qDebug() << "Cinematica inversa:";
    qDebug() << "q1 =" << q[0] << "°";
    qDebug() << "q2 =" << q[1] << "°";
    qDebug() << "q3 =" << q[2] << "°";
    qDebug() << "q4 =" << q[3] << "°";
    qDebug() << "q5 =" << q[4] << "°";
    qDebug() << "q6 =" << q[5] << "°";

    MoverRobotActual();
    AbrirCerrarPinza(1); // Cerrar pinza

}
