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
    interfaz_robot(QWidget *parent = nullptr);
    ~interfaz_robot();

private slots:
    void HabilitarBotones(bool habilitar);
    void startStopCapture();
    void MostrarVideo();
    void MostrarImagen();
    void GuardarImagen();
    void MoverEje();
	void MoverTodosLosEjes();
    void VerificarRango(int valor);
    void iniciarComRobot();

private:
    Ui::interfaz_robotClass ui;
    CVideoAcquisition* camara;      // Cámara para captura de video
    QTimer* timerVideo;
    cv::VideoCapture cap;      // Cámara
    cv::Mat ultimoFrame;
	Ccom_robot* m_robot;
};

