#include <opencv2/opencv.hpp>
#include "camera_calibration.h"
#include <iostream>
#include <vector>
#include <string>
#include "interfaz_robot.h"
#include <fstream>
#include <iomanip>


// CALIBRACIÓN DE CÁMARA
void calibrateCameraFromFiles(const std::vector<std::string>& filenames)
{
    // Ajustar estos parámetros según nuestro patrón
    cv::Size boardSize(9, 6);     // Número de esquinas internas (ancho x alto)
    float squareSize = 10.4f;     // Tamaño real de cada cuadrado en mm 

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
            //cv::imshow("Esquinas detectadas", vis);
            //cv::waitKey(200);
        }
        else
        {
            std::cerr << "No se detectó patrón en " << file << std::endl;
        }
    }

    // Validación mínima
    if (imagePoints.size() < 10)
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

    //// Mostrar corrección de una imagen
    //if (!filenames.empty())
    //{
    //    cv::Mat img = cv::imread(filenames[0]);
    //    if (!img.empty())
    //    {
    //        cv::Mat und;
    //        cv::undistort(img, und, K, D);
    //        //cv::imshow("Original", img);
    //        //cv::imshow("Undistorted", und);
    //        cv::waitKey(0);
    //    }
    //}
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

    // Guardar en fichero
    interfaz_robot robot;
    robot.escribirMatriz("RT_panel_camara.txt", RT);

    std::cout << "Calibración del panel completada. RT_panel_camara =\n" << RT << std::endl;

    // Guardar para comprobaciones
    FileStorage fs("RT_panel_camara.yml", FileStorage::WRITE);
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;
    fs.release();

    cv::Mat vis = img.clone();
    cv::drawChessboardCorners(vis, boardSize, corners, true);
    cv::imshow("Panel detectado", vis);
    cv::waitKey(500);

    return true;
}

// CALIBRACIÓN CÁMARA-ROBOT
cv::Mat RotX(double a) {
    cv::Mat R = cv::Mat::eye(4, 4, CV_64F);
    R.at<double>(1, 1) = cos(a);  R.at<double>(1, 2) = -sin(a);
    R.at<double>(2, 1) = sin(a);  R.at<double>(2, 2) = cos(a);
    return R;
}

cv::Mat RotY(double a) {
    cv::Mat R = cv::Mat::eye(4, 4, CV_64F);
    R.at<double>(0, 0) = cos(a);  R.at<double>(0, 2) = sin(a);
    R.at<double>(2, 0) = -sin(a); R.at<double>(2, 2) = cos(a);
    return R;
}

cv::Mat RotZ(double a) {
    cv::Mat R = cv::Mat::eye(4, 4, CV_64F);
    R.at<double>(0, 0) = cos(a);  R.at<double>(0, 1) = -sin(a);
    R.at<double>(1, 0) = sin(a);  R.at<double>(1, 1) = cos(a);
    return R;
}

cv::Mat Trans(double x, double y, double z) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(0, 3) = x;
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;
    return T;
}

cv::Mat fkBraccio(const double q_deg[6])
{
    double q1 = q_deg[0] * M_PI / 180.0;
    double q2 = q_deg[1] * M_PI / 180.0;
    double q3 = q_deg[2] * M_PI / 180.0;
    double q4 = q_deg[3] * M_PI / 180.0;
    double q5 = q_deg[4] * M_PI / 180.0;

    const double L1 = 76;
    const double L2 = 125;
    const double L3 = 125;
    const double L4 = 60;
    const double L5 = 132;

    cv::Mat RT = //tool base
        RotZ(q5) * Trans(0, 0, L5) *
        RotY(q4) * Trans(0, 0, L4) *
        RotY(q3) * Trans(0, 0, L3) *
        RotY(q2) * Trans(0, 0, L2) *
        RotZ(q1) * Trans(0, 0, L1);

    return RT;
}

bool calibrateCameraRobot(
    int numImages,
    const cv::Size& boardSize,
    float squareSize,
    const cv::Mat& K,
    const cv::Mat& D,
    const std::string& outFile)
{
    // Puntos 3D del tablero
    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    for (int r = 0; r < boardSize.height; r++)
        for (int c = 0; c < boardSize.width; c++)
            objectPoints[0].push_back(cv::Point3f(c * squareSize, r * squareSize, 0));

    // Vectores de rotación/traslación para calibrateHandEye
    std::vector<cv::Mat> R_base2gripper, t_base2gripper;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    for (int i = 1; i <= numImages; ++i)
    {
        std::ostringstream imgFile, poseFile;
        imgFile << "imagen_" << std::setw(2) << std::setfill('0') << i << ".png";
        poseFile << "pose_" << std::setw(2) << std::setfill('0') << i << ".txt";

        // Cargar imagen
        cv::Mat img = cv::imread(imgFile.str());
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Detectar esquinas del tablero
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners);
        //std::cout << "Bool =\n" << found << std::endl;
        if (!found) continue;

        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        // solvePnP devuelve panel camara
        cv::Mat rvec_target2cam, tvec_target2cam; 
        cv::solvePnP(objectPoints[0], corners, K, D, rvec_target2cam, tvec_target2cam);

        cv::Mat R_target2cam_mat;
        cv::Rodrigues(rvec_target2cam, R_target2cam_mat);

        //// Invertir para obtener target to cam
        //cv::Mat R_target2cam_inv = R_target2cam_mat.t();
        //cv::Mat t_target2cam_inv = -R_target2cam_inv * tvec_target2cam;

        R_target2cam.push_back(R_target2cam_mat);
        t_target2cam.push_back(tvec_target2cam);

        // Leer pose del robot (Base a Gripper)
        double q[6];
        std::ifstream f(poseFile.str());
        if (!f.is_open()) continue;
        for (int k = 0; k < 6; k++) f >> q[k];
        f.close();

        cv::Mat RT = fkBraccio(q);   //es gripper base
        cv::Mat RT_inv;
        cv::invert(RT, RT_inv);

        cv::Mat Rb = RT_inv(cv::Range(0, 3), cv::Range(0, 3)).clone();
        cv::Mat tb = RT_inv(cv::Range(0, 3), cv::Range(3, 4)).clone();

        R_base2gripper.push_back(Rb); 
        t_base2gripper.push_back(tb);
    }

    // Calibración hand-eye (eye-to-hand)
    cv::Mat R_cam2base, t_cam2base;
    // primero pasarle el siguiente RT base gripper y luego el anterior RT target camara
    // devuelve cam gripper
    cv::calibrateHandEye( 
        R_base2gripper, t_base2gripper,
        R_target2cam, t_target2cam,
        R_cam2base, t_cam2base, 
        cv::CALIB_HAND_EYE_TSAI);

    // Construir matriz homogénea 4x4 (Cam → Base)
    cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
    R_cam2base.copyTo(RT(cv::Range(0, 3), cv::Range(0, 3)));
    t_cam2base.copyTo(RT(cv::Range(0, 3), cv::Range(3, 4)));

    // Guardar
    interfaz_robot robot;
    robot.escribirMatriz("RT_camara_base.txt", RT);

    // Guardar para comprobaciones
    FileStorage fs("RT_camara_base.yml", FileStorage::WRITE);
    fs << "R_cam2base" << R_cam2base;
    fs << "t_cam2base" << t_cam2base;
    fs.release();

    // Comprobaciones para debug
	FileStorage fs_check("debug_handeye.yml", FileStorage::WRITE);
	fs_check << "R_base2gripper" << R_base2gripper;
	fs_check << "t_base2gripper" << t_base2gripper;
	fs_check << "R_target2cam" << R_target2cam;
	fs_check << "t_target2cam" << t_target2cam;
	fs_check.release();

    std::cout << "RT_camara_base =\n" << RT << std::endl;

    return true;
}










