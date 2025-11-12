#include "procesado.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "interfaz_robot.h"

using namespace cv;
using namespace std;

cv::Mat recortarYReescalar(const cv::Mat& imagenOriginal) {
    // Puntos de la imagen original que queremos mapear
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(480, 173),   // superior izquierda
        cv::Point2f(1330, 183),   // superior derecha
        cv::Point2f(310, 926),   // inferior izquierda
        cv::Point2f(1490, 922)   // inferior derecha
    };

    // Puntos destino (esquinas de la nueva imagen)
    int anchoNuevo = 1920;  // ancho de la nueva imagen
    int altoNuevo = 1080;   // alto de la nueva imagen
    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(anchoNuevo, 0),
        cv::Point2f(0, altoNuevo),
        cv::Point2f(anchoNuevo, altoNuevo)
    };

    // Calcular matriz de homografía inversa
    cv::Mat h = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // Aplicar la transformación
    cv::Mat imagenTransformada;
    cv::warpPerspective(imagenOriginal, imagenTransformada, h, cv::Size(anchoNuevo, altoNuevo));

    return imagenTransformada;
}

//static inline double circularityFromContour(const std::vector<cv::Point>& c) {
//    double area = std::max(1.0, cv::contourArea(c));
//    double peri = cv::arcLength(c, true);
//    if (peri <= 1e-6) return 0.0;
//    return 4.0 * CV_PI * area / (peri * peri); // 1.0 para círculo perfecto
//}

cv::Mat Segmentacion(const cv::Mat& procesada) {
    cv::Mat hsv, canalV, canalS, maskMetal, morf, salida;
    procesada.copyTo(salida);

    // 1. Conversión a HSV
    cv::cvtColor(procesada, hsv, cv::COLOR_BGR2HSV);

    // 2. Extraer canales
    std::vector<cv::Mat> canales;
    cv::split(hsv, canales);
    canalS = canales[1];
    canalV = canales[2];

    // 3. Filtro de mediana para suavizar ruido
    cv::medianBlur(canalV, canalV, 5);
    cv::medianBlur(canalS, canalS, 5);

    // 4. Calcular umbrales adaptativos
    auto calcularUmbralAdaptativo = [](const cv::Mat& canal, double factor = 1.0) -> int {
        double meanVal = cv::mean(canal)[0];
        int umbral = static_cast<int>(meanVal * factor);
        umbral = std::min(std::max(umbral, 30), 220);
        return umbral;
        };

    int umbralBrillo = calcularUmbralAdaptativo(canalV, 1.1);
    int umbralSatur = calcularUmbralAdaptativo(canalS, 0.9);

    std::cout << "Umbral adaptativo - Brillo: " << umbralBrillo
        << "  Saturación: " << umbralSatur << std::endl;

    // 5. Crear máscara por condiciones de brillo y saturación
    cv::Mat maskBrillo, maskSatur;
    cv::threshold(canalV, maskBrillo, umbralBrillo, 255, cv::THRESH_BINARY);
    cv::threshold(canalS, maskSatur, umbralSatur, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(maskBrillo, maskSatur, maskMetal);

    // 6. Suavizado de máscara
    cv::medianBlur(maskMetal, maskMetal, 3);

    // 7. Morfología refinada
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(maskMetal, morf, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(morf, morf, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
    cv::morphologyEx(morf, morf, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

    // 8. Buscar contornos
    std::vector<std::vector<cv::Point>> contornos;
    cv::findContours(morf, contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contornos.empty()) {
        std::cerr << "No se detectaron contornos.\n";
        return morf;
    }

    // 9. Filtrar contornos muy pequeños
    double areaTotal = procesada.rows * procesada.cols;
    std::vector<std::vector<cv::Point>> contornosFiltrados;
    for (auto& c : contornos) {
        if (cv::contourArea(c) > 0.001 * areaTotal)
            contornosFiltrados.push_back(c);
    }

    if (contornosFiltrados.empty()) {
        std::cerr << "Todos los contornos eran demasiado pequeños.\n";
        return morf;
    }

    contornos = contornosFiltrados;

    // 10. Contorno principal (área más grande)
    double areaMax = 0;
    size_t idxMayor = 0;
    for (size_t i = 0; i < contornos.size(); ++i) {
        double area = cv::contourArea(contornos[i]);
        if (area > areaMax) {
            areaMax = area;
            idxMayor = i;
        }
    }

    // 11. Crear imagen binaria final con el contorno principal
    cv::Mat maskFinal = cv::Mat::zeros(morf.size(), CV_8UC1);
    cv::drawContours(maskFinal, contornos, (int)idxMayor, cv::Scalar(255), cv::FILLED);

    // Convertir la máscara binaria a BGR para poder dibujar en color
    cv::Mat maskColor;
    cv::cvtColor(maskFinal, maskColor, cv::COLOR_GRAY2BGR);

    // 12. Calcular y dibujar centroide + ángulo
    cv::Moments M = cv::moments(contornos[idxMayor]);
    if (M.m00 != 0) {
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        std::cout << "Centroide: (" << cx << ", " << cy << ")" << std::endl;

        // Calcular ángulo con arcotangente respecto al origen (0,0)
        double dx = static_cast<double>(cx);
        double dy = static_cast<double>(cy);

        // atan2 devuelve el ángulo en radianes entre el eje X y el vector (dx, dy)
        double anguloRad = atan2(dy, dx);
        double angulo = anguloRad * 180.0 / CV_PI;

        // Normalizar ángulo a [0, 360)
        if (angulo < 0) angulo += 360.0;

        std::cout << "Ángulo (atan2): " << angulo << "°" << std::endl;

        // Dibujar una línea que indique la dirección desde el origen
        cv::Point2f origen(0, 0);
        cv::Point2f centro(cx, cy);
        cv::line(maskColor, origen, centro, cv::Scalar(0, 255, 0), 2); // verde

        // Dibujar el eje angular desde el centroide
        double longitudEje = 100.0;
        cv::Point2f p2(cx + longitudEje * cos(anguloRad), cy + longitudEje * sin(anguloRad));
        cv::line(maskColor, cv::Point(cx, cy), p2, cv::Scalar(255, 0, 0), 2); // azul

        // Dibujar el centroide (blanco)
        cv::circle(maskColor, cv::Point(cx, cy), 6, cv::Scalar(0, 0, 255), -1);

        // Escribir texto (en rojo)
        std::string texto = "Ang:" + std::to_string(static_cast<int>(angulo)) +
            "  (" + std::to_string(cx) + "," + std::to_string(cy) + ")";
        cv::putText(maskColor, texto, cv::Point(cx + 10, cy - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2); // rojo


        // Una vez hay centroide ya en 3D -> cinemática inversa
        // 1. Convertir a Point2d
        cv::Point2d centroide_px(cx, cy);

        // 2. Quitar distorsión (con coeficientes de distorsión)
        cv::Mat cameraMatrix = interfaz_robot::leerMatriz("K.txt");
        cv::Mat distCoeffs = interfaz_robot::leerMatriz("Kc.txt");
        std::vector<cv::Point2d> srcPoints{ centroide_px };
        std::vector<cv::Point2d> undistortedPoints;

        // Función de OpenCV para quitar distorsión
        cv::undistortPoints(srcPoints, undistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

        // Ahora undistortedPoints[0] es el centroide sin distorsión
        cv::Point2d uv_sin_distorsion = undistortedPoints[0];

        // 3. Leer matrices de transformación
        cv::Mat RTpanelCam = interfaz_robot::leerMatriz("RT_panel_camara.txt");
        cv::Mat RTcamRobot = interfaz_robot::leerMatriz("RT_camara_robot.txt");

        // 4. Convertir a coordenadas 3D en el sistema del robot
        cv::Point3d P_robot = interfaz_robot::pixelToWorld3D(uv_sin_distorsion, cameraMatrix, RTpanelCam, RTcamRobot);



        //// 5. Llamar a cinematica inversa
        //interfaz_robot::CinematicaInversa(P_robot.x, P_robot.y, P_robot.z, anguloGlobal, r, z);
    }

    // Devolver la versión en color
    return maskColor;


}







