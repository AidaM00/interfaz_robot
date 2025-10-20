#include <QObject>
#include <QtCore>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//clase que hereda de la clase "hilos"
class CVideoAcquisition : public QThread
{
	Q_OBJECT

private:
	//variables de la clase de acceso privado
	VideoCapture* vidcap; //variable para gestionar la captura de imagenes 
	Mat image; //variable para guardar la última imagen
	QMutex mutex; //variable para bloquear el hilo de lectura de imágenes
	bool capturing; //variable para habilitar/deshabilitar la captura
	bool cameraOK; //variable para indicar si se ha realizado la comunicación con la cámara

private:
	//método que se ejecutará cuando se llame a la función start de esta clase
	void run();

public:
	//constructor por defecto
	CVideoAcquisition(QString videoStreamAddress);

	//destructor
	~CVideoAcquisition();

	//función que devuelve la última imagen
	Mat getImage();
	

	//señales
signals:
	//señal que se produce cada vez que hay una nueva imagen disponible
	void newImageSignal(Mat img);

	//slots
public slots:
	//slot para ejecutar/parar la adquisición de imágenes
	void startStopCapture(bool startCapture);
	void setFoco(int focoSet);
};