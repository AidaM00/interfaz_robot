#ifndef PROCESADO_H
#define PROCESADO_H

#include <opencv2/opencv.hpp>
#include <string>

class Procesado {
public:
    // Constructor: requiere parámetros de calibración
    Procesado(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // Segmenta la pieza y devuelve el centroide (en píxeles)
    cv::Point detectarPieza(const cv::Mat& imagen, cv::Mat& salida);

    // Aplica una transformación de perspectiva (vista superior)
    // "srcPoints" son los 4 puntos en la imagen original
    // "dstSize" define el tamaño del rectángulo final (p.ej. 400x300 píxeles)
    cv::Mat transformarPerspectiva(const cv::Mat& imagen,
        const std::vector<cv::Point2f>& srcPoints,
        cv::Size dstSize);

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
};

#endif
