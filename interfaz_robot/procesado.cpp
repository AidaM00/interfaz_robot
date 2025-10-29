#include "procesado.h"
#include <iostream>

using namespace cv;
using namespace std;

Procesado::Procesado(const Mat& cameraMatrix, const Mat& distCoeffs)
    : cameraMatrix_(cameraMatrix.clone()), distCoeffs_(distCoeffs.clone()) {
}


Point Procesado::detectarPieza(const Mat& imagen, Mat& salida) {
    // === 1. Eliminar distorsión de lente ===
    Mat undistorted;
    undistort(imagen, undistorted, cameraMatrix_, distCoeffs_);

    // === 2. Conversión a gris y suavizado ===
    Mat gray;
    cvtColor(undistorted, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // === 3. Umbral adaptativo (robusto a iluminación) ===
    Mat thresh;
    adaptiveThreshold(gray, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV, 21, 5);

    // === 4. Encontrar contornos ===
    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int bestIdx = -1;
    double bestScore = 0;

    // === 5. Filtrar contorno con forma y área esperadas ===
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area < 1000) continue; // descartar ruido
        Rect bbox = boundingRect(contours[i]);
        double aspect = (double)bbox.width / bbox.height;

        // Filtro para formas alargadas tipo pieza metálica
        if (aspect > 2.0 && aspect < 5.0) {
            double score = area / (aspect * 100.0);
            if (score > bestScore) {
                bestScore = score;
                bestIdx = (int)i;
            }
        }
    }

    salida = undistorted.clone();
    Point centro(-1, -1);

    if (bestIdx >= 0) {
        Moments M = moments(contours[bestIdx]);
        if (M.m00 != 0) {
            centro.x = int(M.m10 / M.m00);
            centro.y = int(M.m01 / M.m00);

            drawContours(salida, contours, bestIdx, Scalar(0, 255, 0), 2);
            circle(salida, centro, 6, Scalar(0, 0, 255), -1);

            cout << "Centroide detectado en: (" << centro.x << ", " << centro.y << ")" << endl;
        }
    }
    else {
        cout << "No se detectó la pieza correctamente." << endl;
    }

    return centro;
}


Mat Procesado::transformarPerspectiva(const Mat& imagen,
    const vector<Point2f>& srcPoints,
    Size dstSize) {
    if (srcPoints.size() != 4) {
        cerr << "Error: Se requieren exactamente 4 puntos fuente para la transformación de perspectiva." << endl;
        return imagen.clone();
    }

    // === 1. Puntos destino (rectángulo regular) ===
    vector<Point2f> dstPoints = {
        Point2f(0, 0),
        Point2f(dstSize.width - 1, 0),
        Point2f(dstSize.width - 1, dstSize.height - 1),
        Point2f(0, dstSize.height - 1)
    };

    // === 2. Calcular homografía ===
    Mat H = getPerspectiveTransform(srcPoints, dstPoints);

    // === 3. Aplicar warpPerspective ===
    Mat imagenWarp;
    warpPerspective(imagen, imagenWarp, H, dstSize);

    return imagenWarp;
}
