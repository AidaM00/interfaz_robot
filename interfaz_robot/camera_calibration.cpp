#include <opencv2/opencv.hpp>
#include "camera_calibration.h"
#include <iostream>
#include <vector>
#include <string>

void calibrateCameraFromFiles(const std::vector<std::string>& filenames)
{
    // Ajustar estos parámetros según nuestro patrón
    cv::Size boardSize(9, 6);     // Número de esquinas internas (ancho x alto)
    float squareSize = 25.0f;     // Tamaño real de cada cuadrado en mm (o la unidad que sea)

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;

    // Puntos 3D del tablero (plano Z=0)
    std::vector<cv::Point3f> obj;
    for (int r = 0; r < boardSize.height; ++r)
        for (int c = 0; c < boardSize.width; ++c)
            obj.emplace_back(c * squareSize, r * squareSize, 0.0f);

    // Procesar cada imagen
    for (const auto& file : filenames)
    {
        cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (img.empty())
        {
            std::cerr << "No se pudo cargar " << file << std::endl;
            continue;
        }

        if (imageSize == cv::Size())
            imageSize = img.size();

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, boardSize, corners);

        if (found)
        {
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            imagePoints.push_back(corners);
            objectPoints.push_back(obj);

            cv::Mat vis;
            cv::cvtColor(img, vis, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(vis, boardSize, corners, found);
            cv::imshow("Esquinas detectadas", vis);
            cv::waitKey(200);
        }
        else
        {
            std::cerr << "No se detectó patrón en " << file << std::endl;
        }
    }

    // Validación mínima
    if (imagePoints.size() < 6)
    {
        std::cerr << "No hay suficientes imágenes válidas para la calibración.\n";
        return;
    }

    // Calibración intrínseca
    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, K, D, rvecs, tvecs);

    std::cout << "\nCalibración completada\n";
    std::cout << "RMS error = " << rms << "\n\n";
    std::cout << "Matriz K =\n" << K << "\n\n";
    std::cout << "Distorsión D =\n" << D << "\n\n";

    // Guardar parámetros
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    fs.release();
    std::cout << "Guardado en camera_calib.yml\n";

    // Mostrar corrección de una imagen
    if (!filenames.empty())
    {
        cv::Mat img = cv::imread(filenames[0]);
        if (!img.empty())
        {
            cv::Mat und;
            cv::undistort(img, und, K, D);
            cv::imshow("Original", img);
            cv::imshow("Undistorted", und);
            cv::waitKey(0);
        }
    }
}
