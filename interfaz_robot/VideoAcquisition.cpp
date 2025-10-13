#include "VideoAcquisition.h"

//constructor
CVideoAcquisition::CVideoAcquisition(QString videoStreamAddress)
{
	//crea el objeto que se utiliza para capturar imagenes
	vidcap = new VideoCapture();

	//conexión con webcam
	cameraOK = vidcap->open(1); // 0 para webcam
	vidcap->set(CAP_PROP_AUTOFOCUS, 0); // desactivar autofocus
	vidcap->set(CAP_PROP_FOCUS, 21); // establecer foco manual
	//conexión con cámara ip
	//cameraOK = vidcap->open(videoStreamAddress.toStdString());

	//inicialización para no empezar a capturar imágenes
	capturing = false;
}

//destructor
CVideoAcquisition::~CVideoAcquisition(void)
{
	//parar captura
	startStopCapture(false);

	//terminar el hilo
	this->terminate();

	//liberar la memoria del objeto
	vidcap->release();
}

//funcion para empezar a capturar imagenes o parar
void CVideoAcquisition::startStopCapture(bool startCapture)
{
	//si se ha podido abrir la cámara
	if (cameraOK)
	{
		//si hay que capturar
		if (startCapture)
		{
			//se habilita la captura
			capturing = true;
			//se lanza el hilo de captura
			this->start();
		}
		else
			//si no hay que capturar, se deshabilita la captura
			capturing = false;
	}
	else
		//si la cámara no se ha podido abrir, se lanza un mensaje de error
		qDebug() << QString("ERROR: La cámara no ha podido ser abierta");
}

//función que captura imágenes cada 10ms
void CVideoAcquisition::run(void)
{
	//mientras haya que capturar
	while (capturing)
	{
		//se bloquea el hilo
		mutex.lock();
		//se captura la imagen y se guarda en la variable image
		vidcap->read(image);
		//se lanza la señal de que ya hay disponible una nueva imagen
		emit newImageSignal(image);
		//se desbloquea el hilo
		mutex.unlock();
		//se esperan 10ms para realizar la siguiente captura
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}

//función que devuelve la ultima imagen obtenida
Mat CVideoAcquisition::getImage()
{
	//se bloquea el hilo
	mutex.lock();
	//se guarda la ultima imagen capturada
	Mat img = image.clone();
	//se desbloquea el hilo
	mutex.unlock();
	//re devuelve la imagen
	return img;
}