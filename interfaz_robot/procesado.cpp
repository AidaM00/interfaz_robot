#include "procesado.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

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


cv::Mat Segmentacion(const cv::Mat& procesada) {
    if (procesada.empty()) {
        std::cerr << "Error: imagen vacía recibida en Segmentacion.\n";
        return cv::Mat();
    }

    // Escala de grises y suavizado
    cv::Mat gris;
    cv::cvtColor(procesada, gris, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gris, gris, cv::Size(5, 5), 0);
    cv::medianBlur(gris, gris, 5);

    // Umbral adaptativo
    cv::Mat binaria;
    cv::adaptiveThreshold(
        gris, binaria, 255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY_INV, 21, 3
    );

    // Morfología (limpieza)
    cv::morphologyEx(binaria, binaria, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
    cv::morphologyEx(binaria, binaria, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(binaria, binaria,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

    // Contornos
    std::vector<std::vector<cv::Point>> contornos;
    cv::findContours(binaria.clone(), contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat salidaBinaria = cv::Mat::zeros(binaria.size(), CV_8UC1);
    const double areaMinima = 14000.0;

    std::vector<cv::RotatedRect> cajasRotadas;
    std::vector<double> areasValidas;
    std::vector<std::vector<cv::Point>> contornosValidos;

    for (const auto& c : contornos) {
        double area = cv::contourArea(c);
        if (area >= areaMinima) {
            cv::drawContours(salidaBinaria, std::vector<std::vector<cv::Point>>{c}, -1, 255, cv::FILLED);
            cv::RotatedRect box = cv::minAreaRect(c);
            cajasRotadas.push_back(box);
            areasValidas.push_back(area);
            contornosValidos.push_back(c);
        }
    }

    std::cout << "Objetos detectados: " << cajasRotadas.size() << std::endl;

    // Imagen final color
    cv::Mat salidaFinal;
    cv::cvtColor(salidaBinaria, salidaFinal, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < cajasRotadas.size(); ++i) {
        // Contorno suavizado
        std::vector<cv::Point> contornoSuave;
        cv::approxPolyDP(contornosValidos[i], contornoSuave, 2.0, true);

        // Contorno rojo
        cv::drawContours(salidaFinal, std::vector<std::vector<cv::Point>>{contornoSuave}, -1, cv::Scalar(0, 0, 255), 2);

        // Rectángulo rotado azul
        cv::Point2f vertices[4];
        cajasRotadas[i].points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(salidaFinal, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
        }

        // Centroide
        cv::Moments m = cv::moments(contornosValidos[i]);
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);
        cv::circle(salidaFinal, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);

        // Texto área + ángulo
        std::string texto = "A=" + std::to_string(static_cast<int>(areasValidas[i])) +
            " ang=" + std::to_string(static_cast<int>(cajasRotadas[i].angle));
        cv::putText(salidaFinal, texto,
            cv::Point(static_cast<int>(cajasRotadas[i].center.x), static_cast<int>(cajasRotadas[i].center.y) - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 255, 255), 2);

        std::cout << "Objeto " << i + 1
            << " -> Area=" << static_cast<int>(areasValidas[i])
            << "  Centro(" << cx << "," << cy << ")"
            << "  Angulo=" << cajasRotadas[i].angle
            << std::endl;
    }

    return salidaFinal;
}