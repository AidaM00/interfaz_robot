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
struct PuntosProcesados {
    cv::Point2d centro;
    cv::Point2d lejano;
};

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
    void startStopCapture(bool capturando);
    void MostrarVideo();
    void MostrarFrame(cv::Mat frame, Point2f centro, Point2f lejano);
    void GuardarImagen();
    void MoverEje();
    void MoverTodosLosEjes();
	void MoverPosInterm();
    void VerificarRango(int valor);
    void iniciarComRobot();
    void ActualizarInterfaz();
    void CalibrarCamara();
	void CalibrarPanel();
	void GuardarImagenYPose();
	void CalibrarCamaraRobot();
    void CinematicaInversa(double cx, double cy, double cz, double angulo);
    void AbrirCerrarPinza(int accion);
    void TrasladarPieza();
    void MoverACota();
    void MoverEje(int indexEje, int grados);
    void MoverTodosLosEjes(int* angulos);
    void getNewFrame();
    PuntosProcesados ComenzarProcesado(Mat img);
    void onComenzar();
    void onCoger();

private:
    Ui::interfaz_robotClass ui;
    CVideoAcquisition* camara;      // Cámara para captura de video
    QTimer* timerVideo;
    cv::VideoCapture cap;      // Cámara
    cv::Mat ultimoFrame;
    Ccom_robot* m_robot;
    
    // Constantes del robot (mm)
    const double a1 = 76;
    const double a2 = 125;
    const double a3 = 125;
    const double a4 = 60;
    const double a5 = 132;

    // Ángulos actuales (grados)
    double q[6] = { 0, 0, 0, 0, 0, 0 };  // Ángulos actuales del robot en grados

    int contador = 1; // Contador global para los archivos (calib cámara-robot)
    // Parámetros del panel
    cv::Size boardSize=cv::Size(9, 6);     // Esquinas internas
    float m_squareSize = 10.4f;     // mm
    bool m_capturando = false;
    bool m_comenzarProcesado = false;
	bool m_cogerPieza = false;
    bool m_mostrarFrameCongelado = false;

    PuntosProcesados m_ultimosPuntos;
    cv::Mat m_ultimoFrame;

};

