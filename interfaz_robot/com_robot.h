#pragma once

#include <QtSerialPort/qserialport.h>

class Ccom_robot : public QObject
{
	Q_OBJECT

public:
	Ccom_robot(int com);
	~Ccom_robot();
	void mover(int eje, int angulo);
	void enviarComando(const QString& comando);
	void obtenerPosicionActual();

protected:
	QSerialPort m_serial;
};
