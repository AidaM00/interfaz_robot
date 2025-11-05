#ifndef PROCESADO_H
#define PROCESADO_H

#include <opencv2/opencv.hpp>
#include <string>

cv::Mat recortarYReescalar(const cv::Mat& imagenOriginal);
cv::Mat Segmentacion(const cv::Mat& procesada);

#endif
