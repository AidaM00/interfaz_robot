#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Funci�n para calibrar la c�mara a partir de una lista de im�genes
// filenames: vector con rutas a las im�genes de calibraci�n
void calibrateCameraFromFiles(const std::vector<std::string>& filenames);
bool calibratePanel(const std::string& imgFile,
    const cv::Mat& K, const cv::Mat& D,
    const cv::Size& boardSize,
    float squareSize,
    const std::string& outFile);

#endif // CAMERA_CALIBRATION_H
