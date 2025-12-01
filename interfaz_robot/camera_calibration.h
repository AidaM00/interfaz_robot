#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Función para calibrar la cámara a partir de una lista de imágenes
// filenames: vector con rutas a las imágenes de calibración
void calibrateCameraFromFiles(const std::vector<std::string>& filenames);
bool calibratePanel(const std::string& imgFile,
    const cv::Mat& K, const cv::Mat& D,
    const cv::Size& boardSize,
    float squareSize,
    const std::string& outFile);
bool calibrateCameraRobot(int numImages, const cv::Size& boardSize, float squareSize,
    const cv::Mat& K, const cv::Mat& D, const std::string& outFile);
cv::Mat fkBraccio(const double q_deg[6]);
cv::Mat RotY(double a); 
cv::Mat RotZ(double a);
cv::Mat Trans(double x, double y, double z);

#endif // CAMERA_CALIBRATION_H
