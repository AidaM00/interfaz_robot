#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Función para calibrar la cámara a partir de una lista de imágenes
// filenames: vector con rutas a las imágenes de calibración
void calibrateCameraFromFiles(const std::vector<std::string>& filenames);

#endif // CAMERA_CALIBRATION_H
