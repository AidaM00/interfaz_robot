#pragma once

#include <QtWidgets/QMainWindow>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "VideoAcquisition.h"
#include "ui_interfaz_robot.h"
#include "com_robot.h"
#include "procesado.h"

namespace Ui { class interfaz_robot; }
namespace fs = std::filesystem;

class interfaz_robot : public QMainWindow
{
    Q_OBJECT

public:
    interfaz_robot(QWidget* parent = nullptr);
    ~interfaz_robot();
    static void escribirMatriz(const std::string& nombreArchivo, const cv::Mat& M);
    static cv::Mat leerMatriz(const std::string& nombreArchivo);
    static cv::Point3d pixelToWorld3D(const cv::Point2d& uv,
        const cv::Mat& K,
        const cv::Mat& RTpanelCam,
        const cv::Mat& RTcamRobot);

private slots:
    void HabilitarBotones(bool habilitar);
    void startStopCapture();
    void MostrarVideo();
    void GuardarImagen();
    void MoverEje();
    void MoverTodosLosEjes();
    void VerificarRango(int valor);
    void iniciarComRobot();
    //void ActualizarPosicionRobot();
    void Directa();
    void CalibrarCamara();
	void CalibrarPanel();
	void GuardarImagenYPose();
	void CalibrarCamaraRobot();
    cv::Mat ProcesarImagen();

private:
    Ui::interfaz_robotClass ui;
    CVideoAcquisition* camara;      // Cámara para captura de video
    QTimer* timerVideo;
    cv::VideoCapture cap;      // Cámara
    cv::Mat ultimoFrame;
    Ccom_robot* m_robot;
    

    double q[6] = { 0,0,0,0,0,0 };   // Ángulos actuales del robot en grados
    int contador = 1; // Contador global para los archivos (calib cámara-robot)
    // Parámetros del panel
    cv::Size boardSize=cv::Size(9, 6);     // Esquinas internas
    float m_squareSize = 10.4f;     // mm

};

