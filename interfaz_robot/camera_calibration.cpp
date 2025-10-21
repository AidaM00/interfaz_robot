#include <opencv2/opencv.hpp>
#include "camera_calibration.h"
#include <iostream>
#include <vector>
#include <string>
#include "interfaz_robot.h"

// CALIBRACIÓN DE CÁMARA
void calibrateCameraFromFiles(const std::vector<std::string>& filenames)
{
    // Ajustar estos parámetros según nuestro patrón
    cv::Size boardSize(9, 6);     // Número de esquinas internas (ancho x alto)
    float squareSize = 10.40f;     // Tamaño real de cada cuadrado en mm (o la unidad que sea)

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;

    // Puntos 3D del tablero (plano Z=0)
    std::vector<cv::Point3f> obj;
    for (int r = 0; r < boardSize.height; ++r)
        for (int c = 0; c < boardSize.width; ++c)
            obj.push_back(cv::Point3f(c * squareSize, r * squareSize, 0.0f));

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
    if (imagePoints.size() < 20)
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
    interfaz_robot robot;
    robot.escribirMatriz("K.txt", K);


    robot.escribirMatriz("Kc.txt", D);
    cv::FileStorage fs("camera_calib.txt", cv::FileStorage::WRITE);
    fs << "K" << K;
    fs << "D" << D;
    fs.release();
    std::cout << "Guardado en camera_calib.txt\n";

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

// CALIBRACIÓN DE PANEL
bool calibratePanel(const std::string& imgFile, const cv::Mat& K, const cv::Mat& D,
    const cv::Size& boardSize, float squareSize, const std::string& outFile)
{
    cv::Mat img = cv::imread(imgFile);
    if (img.empty())
    {
        std::cerr << "Error: no se pudo cargar la imagen del panel.\n";
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, boardSize, corners);

    if (!found)
    {
        std::cerr << "No se detectó el patrón en la imagen.\n";
        return false;
    }

    cv::cornerSubPix(gray, corners, cv::Size(9, 9), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

    std::vector<cv::Point3f> obj;
    for (int r = 0; r < boardSize.height; ++r)
        for (int c = 0; c < boardSize.width; ++c)
            obj.emplace_back(c * squareSize, r * squareSize, 0.0f);

    // Pose
    cv::Mat rvec, tvec;
    cv::solvePnP(obj, corners, K, D, rvec, tvec);

    // Matriz R 3x3
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Matriz homogénea 4x4 RT
    cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(RT(cv::Range(0, 3), cv::Range(0, 3)));
    tvec.copyTo(RT(cv::Range(0, 3), cv::Range(3, 4)));

    // Guardar RT en fichero
    cv::FileStorage fs(outFile, cv::FileStorage::WRITE);
    fs << "RT" << RT;
    fs.release();

    std::cout << "Calibración del panel completada. RT =\n" << RT << std::endl;

    cv::Mat vis = img.clone();
    cv::drawChessboardCorners(vis, boardSize, corners, true);
    cv::imshow("Panel detectado", vis);
    cv::waitKey(500);

    return true;
}