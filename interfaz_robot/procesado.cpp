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
        cv::Point2f(468, 252),    // Superior izquierda
        cv::Point2f(1318, 267),   // Superior derecha
        cv::Point2f(271, 1048),    // Inferior izquierda
        cv::Point2f(1487, 1044)    // Inferior derecha
    };

    // Puntos destino (esquinas de la nueva imagen)
    int anchoNuevo = 1920;  // Ancho de la nueva imagen
    int altoNuevo = 1080;   // Alto de la nueva imagen
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

    // Guardar la imagen procesada
    cv::imwrite("pieza_recortada_reescalada.png", imagenTransformada);

    return imagenTransformada;
}


// Función que localiza centroide y punto lejano
int contador_guardar = 0;
void LocalizarPieza(const cv::Mat& procesada, cv::Point2f &centro, cv::Point2f &ptoLejano)
{
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

   /* std::cout << "Umbral adaptativo - Brillo: " << umbralBrillo
        << "  Saturacion: " << umbralSatur << std::endl;*/

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

    // 9. Filtrar contornos muy pequeños
    double areaTotal = procesada.rows * procesada.cols;
    std::vector<std::vector<cv::Point>> contornosFiltrados;
    for (auto& c : contornos) {
        if (cv::contourArea(c) > 0.001 * areaTotal)
            contornosFiltrados.push_back(c);
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
    // Guardar la máscara binaria segmentada
        std::ostringstream nombreArchivo;
    nombreArchivo << "pieza_segmentada_binaria"
        << std::setw(3) << std::setfill('0') << contador_guardar
        << ".png";
    cv::imwrite(nombreArchivo.str(), maskFinal);
    contador_guardar++; // Incrementar contador para el próximo guardado


    // Convertir la máscara binaria a BGR para poder dibujar en color
    cv::Mat maskColor;
    cv::cvtColor(maskFinal, maskColor, cv::COLOR_GRAY2BGR);

    // Dibujar contorno principal en color (rojo)
    cv::drawContours(maskColor, contornos, (int)idxMayor, cv::Scalar(0, 0, 255), 2);

    // 12. Calcular y dibujar centroide
    cv::Moments M = cv::moments(contornos[idxMayor]);
    if (M.m00 != 0) {
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        //std::cout << "Centroide: (" << cx << ", " << cy << ")" << std::endl;

        centro = cv::Point2f(cx, cy);


        // Dibujar el centroide 
        cv::circle(maskColor, cv::Point(cx, cy), 6, cv::Scalar(0, 0, 255), -1);

        // Filtrar el contorno para quedarse solo con la parte dentro del círculo 
        int radio = 100;
        cv::circle(maskColor, centro, radio, cv::Scalar(0, 255, 0), 2); // Dibujar círculo verde

        // Recoger puntos del contorno principal dentro del círculo
        std::vector<cv::Point> puntosDentro;
        for (const cv::Point& p : contornos[idxMayor]) {
            float dx = p.x - cx;
            float dy = p.y - cy;
            if (dx * dx + dy * dy <= radio * radio) {
                puntosDentro.push_back(p);
            }
        }
        //std::cout << "Puntos del contorno dentro del círculo: " << puntosDentro.size() << std::endl;

        // Si hay puntos, generar un contorno cerrado usando convexHull
        std::vector<cv::Point> contornoDentro;
        if (!puntosDentro.empty()) {
            cv::convexHull(puntosDentro, contornoDentro);  // Asegura un contorno cerrado
        }
        std::vector<cv::Point2f> contornoDenso;
        float deltax, deltay, m;
        for (int i = 1; i < contornoDentro.size(); i++)  // Recorrer el contorno
        {
            deltay = contornoDentro.at(i).y - contornoDentro.at(i - 1).y;
            deltax = contornoDentro.at(i).x - contornoDentro.at(i - 1).x;
            m = deltay / deltax;
            if (abs(m) < 1)
            {
                int signo = 1;
                if (deltax < 0) signo = -1;
                for (float x = contornoDentro.at(i - 1).x; ((x < contornoDentro.at(i).x) && (signo > 0)) || ((x > contornoDentro.at(i).x) && (signo < 0)); x += signo)
                {
                    float y = m * (x - contornoDentro.at(i - 1).x) + contornoDentro.at(i - 1).y;
                    contornoDenso.push_back(cv::Point2f(x, y));
                }
            }
            else
            {
                int signo = 1;
                if (deltay < 0) signo = -1;
                for (float y = contornoDentro.at(i - 1).y; ((y < contornoDentro.at(i).y) && (signo > 0)) || ((y > contornoDentro.at(i).y) && (signo < 0)); y += signo)
                {
                    float x = (y - contornoDentro.at(i - 1).y) / m + contornoDentro.at(i - 1).x;
                    contornoDenso.push_back(cv::Point2f(x, y));
                }
            }
        }

        int numPtos = contornoDenso.size();
        std::vector<cv::Point3f> puntosMinimos;
        float maxDistance = 0.0f;
        //cv::Point ptoLejano;
        for (int i = 0; i < numPtos; i++)
        {
            int indexAnt = i - 1;
            if (indexAnt < 0) indexAnt = numPtos - 1;
            int indexPost = i + 1;
            if (indexPost >= numPtos) indexPost = 0;
            cv::Point2f distPost = cv::Point2f(contornoDenso[indexPost].x, contornoDenso[indexPost].y) - centro;
            cv::Point2f distAnt = cv::Point2f(contornoDenso[indexAnt].x, contornoDenso[indexAnt].y) - centro;
            cv::Point2f disti = cv::Point2f(contornoDenso[i].x, contornoDenso[i].y) - centro;
            float moduloi = std::sqrt(disti.x * disti.x + disti.y * disti.y);
            float moduloPost = std::sqrt(distPost.x * distPost.x + distPost.y * distPost.y);
            float moduloAnt = std::sqrt(distAnt.x * distAnt.x + distAnt.y * distAnt.y);
            if ((moduloi < moduloAnt) && (moduloi < moduloPost))
            {
                cv::circle(maskColor, contornoDenso[i], 6, cv::Scalar(0, 255, 255), -1);
                bool pto_introducido = false;
                for (int j = 0; j < puntosMinimos.size(); j++)
                {
                    if (moduloi < puntosMinimos[j].z)
                    {
                        puntosMinimos.insert(puntosMinimos.begin() + j, cv::Point3f(contornoDenso[i].x, contornoDenso[i].y, moduloi));
                        pto_introducido = true;
                        break;
                    }
                }
                if(!pto_introducido)
                    puntosMinimos.push_back(cv::Point3f(contornoDenso[i].x, contornoDenso[i].y, moduloi));
            }
        }
        if (puntosMinimos.size() > 1)
            ptoLejano = cv::Point2f(puntosMinimos.at(1).x, puntosMinimos.at(1).y);

        // Dibujar ptoLejano 
        cv::circle(maskColor, ptoLejano, 6, cv::Scalar(0, 0, 255), -1);
        //// Carpeta donde guardar
        //fs::path rutaGuardar = fs::current_path();
        //// Generar nombre con sufijo incremental
        //std::ostringstream nombreArchivo;
        //nombreArchivo << "pieza_localizada_"
        //    << std::setw(2) << std::setfill('0') << contador_guardar
        //    << ".png";
        //fs::path rutaSalida = rutaGuardar / nombreArchivo.str();
        //std::cout << "Guardando imagen segmentada en: " << rutaSalida << std::endl;
        //// Guardar imagen
        //cv::imwrite(rutaSalida.string(), maskColor);
        //// Incrementar contador para la siguiente imagen
        //contador_guardar++;
    }
}
