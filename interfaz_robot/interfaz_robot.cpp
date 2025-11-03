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


namespace fs = std::filesystem;

double q[6] = { 0,0,0,0,0,0 };   // Ángulos actuales del robot en grados
int contador = 1; // Contador global para los archivos (calib cámara-robot)
// Parámetros del panel
cv::Size boardSize(9, 6);     // Esquinas internas
float squareSize = 10.4f;     // mm

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
    //// Lista de archivos de calibración
    std::vector<std::string> archivos = {
     "calib_camara_01.png","calib_camara_02.png", "calib_camara_03.png" "calib_camara_04.png", "calib_camara_05.png",
    "calib_camara_06.png", "calib_camara_07.png", "calib_camara_08.png", "calib_camara_09.png", "calib_camara_10.png",
    "calib_camara_11.png", "calib_camara_12.png", "calib_camara_13.png", "calib_camara_14.png", "calib_camara_15.png",
     "calib_camara_16.png", "calib_camara_17.png", "calib_camara_18.png", "calib_camara_19.png", "calib_camara_20.png",
    "calib_camara_21.png", "calib_camara_22.png", "calib_camara_23.png", "calib_camara_24.png"
   
    
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
    bool ok = calibrateCameraRobot(numImages, boardSize, squareSize, K, D, outFile);

    if (ok)
        qDebug() << "Calibracion completa. RT camara-robot guardada";
    else
        qDebug() << "Error: La calibracion camara-robot fallo.";
}

//void interfaz_robot::Limpiar()
//{
//    ui.lbimagen->clear();  // Limpiar la QLabel
//}

cv::Mat interfaz_robot::ProcesarImagen() {
    namespace fs = std::filesystem;
    fs::path rutaEjecutable = fs::current_path();
    std::vector<fs::path> imagenes;

    for (const auto& entry : fs::directory_iterator(rutaEjecutable)) {
        std::string nombre = entry.path().filename().string();
        if (nombre.rfind("pieza_", 0) == 0 && nombre.find("segmentada") == std::string::npos && entry.path().extension() == ".png") {
            imagenes.push_back(entry.path());
        }
    }

    if (imagenes.empty()) {
        std::cout << "No se encontraron imágenes que empiecen por 'pieza_'.\n";
        return cv::Mat();
    }

    std::sort(imagenes.begin(), imagenes.end());
    cv::Mat ultimaProcesada;

    for (const auto& rutaImagen : imagenes) {
        std::cout << "Procesando: " << rutaImagen << std::endl;
        cv::Mat img = cv::imread(rutaImagen.string());
        if (img.empty()) {
            std::cerr << "No se pudo cargar " << rutaImagen << "\n";
            continue;
        }

        // === 1. Recortar y reescalar ===
        cv::Mat procesada = recortarYReescalar(img);
        if (procesada.empty()) {
            std::cerr << "Error: resultado vacío tras recortarYReescalar.\n";
            continue;
        }

        // === 2. Convertir a escala de grises ===
        cv::Mat gris;
        cv::cvtColor(procesada, gris, cv::COLOR_BGR2GRAY);

        // === 3. Filtro de suavizado ===
        cv::Mat suavizada;
        cv::GaussianBlur(gris, suavizada, cv::Size(5, 5), 0);
        cv::medianBlur(suavizada, suavizada, 5);

        // === 4. Umbral adaptativo ===
        cv::Mat binaria;
        cv::adaptiveThreshold(
            suavizada, binaria, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY_INV, 21, 3
        );

        // === 5. Morfología para limpiar ruido ===
        cv::Mat kernelCierre = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(binaria, binaria, cv::MORPH_CLOSE, kernelCierre, cv::Point(-1, -1), 1);
        cv::Mat kernelApertura = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(binaria, binaria, cv::MORPH_OPEN, kernelApertura, cv::Point(-1, -1), 1);

        // === 6. Detección de contornos ===
        std::vector<std::vector<cv::Point>> contornos;
        cv::findContours(binaria.clone(), contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // === 7. Filtrar contornos por área y bordes ===
        cv::Mat plano = cv::Mat::zeros(binaria.size(), CV_8UC1);
        double areaMinima = procesada.cols * procesada.rows * 0.001;
        std::vector<std::vector<cv::Point>> contornosValidos;
        std::vector<cv::Rect> bboxes;

        for (const auto& c : contornos) {
            double area = cv::contourArea(c);
            if (area < areaMinima) continue;
            cv::Rect bbox = cv::boundingRect(c);
            if (bbox.x <= 1 || bbox.y <= 1 || bbox.x + bbox.width >= procesada.cols - 1 || bbox.y + bbox.height >= procesada.rows - 1)
                continue;
            contornosValidos.push_back(c);
            bboxes.push_back(bbox);
        }

        // Parámetros de fusión (ajustables)
        const double distanciaMax = 25.0; // px máximo para considerar fusión
        const double maxForegroundFrac = 0.30; // fracción máxima de foreground en la línea de conexión
        const int samplesAlongLine = 20; // cantidad de muestras a tomar a lo largo de la línea

        // Helper: calcula la distancia mínima entre 2 contornos y los puntos más cercanos (pA, pB)
        auto contourMinDistAndPoints = [](const std::vector<cv::Point>& A, const std::vector<cv::Point>& B, cv::Point& pA_out, cv::Point& pB_out) -> double {
            double minD = std::numeric_limits<double>::max();
            for (const cv::Point& pa : A) {
                for (const cv::Point& pb : B) {
                    double d = cv::norm(pa - pb);
                    if (d < minD) {
                        minD = d;
                        pA_out = pa;
                        pB_out = pb;
                    }
                }
            }
            return minD;
            };

        // === 8. Mejor unión de contornos: criterios múltiples y verificación por línea ===
        cv::Mat planoUnion = cv::Mat::zeros(binaria.size(), CV_8UC1);
        cv::drawContours(planoUnion, contornosValidos, -1, 255, cv::FILLED);
        std::vector<int> merge_partner(contornosValidos.size(), -1);

        for (size_t i = 0; i < contornosValidos.size(); ++i) {
            for (size_t j = i + 1; j < contornosValidos.size(); ++j) {
                cv::Rect bi = bboxes[i];
                cv::Rect bj = bboxes[j];
                cv::Rect unionBB = bi | bj;
                double diagMax = std::sqrt(std::max(bi.width * bi.width + bi.height * bi.height, bj.width * bj.width + bj.height * bj.height));

                cv::Point2d ci(bi.x + bi.width / 2.0, bi.y + bi.height / 2.0);
                cv::Point2d cj(bj.x + bj.width / 2.0, bj.y + bj.height / 2.0);
                double centroDist = cv::norm(ci - cj);
                if (centroDist > (diagMax * 1.5 + distanciaMax * 2.0)) {
                    continue;
                }

                cv::Point pA, pB;
                double minD = contourMinDistAndPoints(contornosValidos[i], contornosValidos[j], pA, pB);
                if (minD > distanciaMax) continue;

                int fgCount = 0;
                int totalSamples = std::max(2, samplesAlongLine);
                for (int s = 0; s <= totalSamples; ++s) {
                    double t = static_cast<double>(s) / totalSamples;
                    int x = static_cast<int>(std::round(pA.x + t * (pB.x - pA.x)));
                    int y = static_cast<int>(std::round(pA.y + t * (pB.y - pA.y)));
                    x = std::clamp(x, 0, binaria.cols - 1);
                    y = std::clamp(y, 0, binaria.rows - 1);
                    if (binaria.at<uchar>(y, x) > 0) fgCount++;
                }

                double fgFrac = static_cast<double>(fgCount) / (totalSamples + 1);
                bool acceptMerge = false;
                if (fgFrac <= maxForegroundFrac) {
                    acceptMerge = true;
                }
                else {
                    if (minD < std::max(3.0, diagMax * 0.08)) {
                        acceptMerge = true;
                    }
                }

                if (acceptMerge) {
                    if (merge_partner[i] == -1 && merge_partner[j] == -1) {
                        int idx = static_cast<int>(i);
                        merge_partner[i] = idx;
                        merge_partner[j] = idx;
                    }
                    else if (merge_partner[i] != -1 && merge_partner[j] == -1) {
                        merge_partner[j] = merge_partner[i];
                    }
                    else if (merge_partner[j] != -1 && merge_partner[i] == -1) {
                        merge_partner[i] = merge_partner[j];
                    }
                    else {
                        int a = merge_partner[i];
                        int b = merge_partner[j];
                        if (a != b) {
                            for (size_t k = 0; k < merge_partner.size(); ++k)
                                if (merge_partner[k] == b)
                                    merge_partner[k] = a;
                        }
                    }
                    cv::line(planoUnion, pA, pB, 255, static_cast<int>(std::max(3.0, diagMax * 0.02)));
                }
            }
        }

        int nextCluster = static_cast<int>(contornosValidos.size());
        for (size_t i = 0; i < contornosValidos.size(); ++i) {
            if (merge_partner[i] == -1) {
                merge_partner[i] = static_cast<int>(i);
            }
        }

        std::map<int, std::vector<int>> clusters;
        for (size_t i = 0; i < merge_partner.size(); ++i) {
            clusters[merge_partner[i]].push_back(static_cast<int>(i));
        }

        cv::Mat planoClusters = cv::Mat::zeros(binaria.size(), CV_8UC1);
        for (auto& kv : clusters) {
            for (int idx : kv.second) {
                cv::drawContours(planoClusters, contornosValidos, idx, 255, cv::FILLED);
            }
        }

        cv::Mat kernelDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::dilate(planoClusters, planoClusters, kernelDilate);
        cv::Mat kernelClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(planoClusters, planoClusters, cv::MORPH_CLOSE, kernelClose);

        // === 9. Dibujar contornos finales sobre imagen procesada ===
        std::vector<std::vector<cv::Point>> contornosFinales;
        cv::findContours(planoClusters.clone(), contornosFinales, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Mat salidaColor;
        cv::cvtColor(planoClusters, salidaColor, cv::COLOR_GRAY2BGR);
        cv::drawContours(salidaColor, contornosFinales, -1, cv::Scalar(0, 0, 255), 2);

        // === 10. Guardar resultado ===
        fs::path rutaColor = rutaImagen.parent_path() / (rutaImagen.stem().string() + "_segmentada.png");
        cv::imwrite(rutaColor.string(), salidaColor);
        std::cout << "Guardada la imagen segmentada: " << rutaColor << std::endl;

        ultimaProcesada = salidaColor.clone();
    }

    return ultimaProcesada;
}






























