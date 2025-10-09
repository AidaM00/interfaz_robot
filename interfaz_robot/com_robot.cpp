#include "com_robot.h"
#include <QDebug>	

Ccom_robot::Ccom_robot(int com)
{
	m_serial.setPortName("COM" + QString::number(com));
	m_serial.setBaudRate(QSerialPort::Baud9600);
	//m_serial.setDataBits(QSerialPort::Data8);
	//m_serial.setStopBits(QSerialPort::OneStop);
	//m_serial.setFlowControl(QSerialPort::NoFlowControl);
	if (!m_serial.isOpen())
	{
		if (!m_serial.open(QIODevice::ReadWrite))
			qDebug() << "Error al abrir el puerto serie";
		else
			qDebug() << "Puerto serie abierto";
	}
	else
		qDebug() << "El puerto serie ya está abierto";
}

Ccom_robot::~Ccom_robot()
{

}

void Ccom_robot::mover(int eje, int angulo)
{
	QString comando;
	comando = "#m" + QString::number(eje) + "-" + QString::number(angulo) + "*";
	m_serial.write(comando.toLatin1().data());
}

void Ccom_robot::enviarComando(const QString& comando)
{
	if (!m_serial.isOpen()) {
		qDebug() << "Error: el puerto serie no está abierto.";
		return;
	}

	m_serial.write(comando.toLatin1().data());
	m_serial.flush();  // asegura que se envíe
	qDebug() << "Comando enviado:" << comando;
}
