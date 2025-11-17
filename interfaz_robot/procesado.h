#ifndef PROCESADO_H
#define PROCESADO_H

#include <opencv2/opencv.hpp>
#include <string>
#include "interfaz_robot.h"

cv::Mat recortarYReescalar(const cv::Mat& imagenOriginal);
void LocalizarPieza(const cv::Mat& procesada, cv::Point2f& centro, cv::Point2f& ptoLejano);
cv::Point3d Inversa(const cv::Point& punto_px);


#endif
