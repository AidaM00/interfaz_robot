#pragma once

#include <QtWidgets/QMainWindow>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "VideoAcquisition.h"
#include "ui_interfaz_robot.h"
#include "com_robot.h"

namespace Ui { class interfaz_robot; }

class interfaz_robot : public QMainWindow
{
    Q_OBJECT

public:
    interfaz_robot(QWidget* parent = nullptr);
    ~interfaz_robot();
    static void escribirMatriz(const std::string& nombreArchivo, const cv::Mat& M);
    static cv::Mat leerMatriz(const std::string& nombreArchivo);

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

private:
    Ui::interfaz_robotClass ui;
    CVideoAcquisition* camara;      // Cámara para captura de video
    QTimer* timerVideo;
    cv::VideoCapture cap;      // Cámara
    cv::Mat ultimoFrame;
    Ccom_robot* m_robot;

};

