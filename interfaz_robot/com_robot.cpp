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
    if (m_serial.isOpen())
        m_serial.close();
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

//std::vector<int> Ccom_robot::obtenerPosicionActual()
//{
//    std::vector<int> posiciones(6, 0); // Inicializamos con 6 ceros por defecto
//
//    if (!m_serial.isOpen()) {
//        qDebug() << "Puerto no abierto, no se puede leer posición.";
//        return posiciones;
//    }
//
//    // Enviar el comando que el robot usa para devolver su posición actual
//    QByteArray comando = "#g*";
//    m_serial.write(comando);
//    m_serial.flush();
//
//    // Esperar respuesta del robot
//    if (!m_serial.waitForReadyRead(500)) {
//        qDebug() << "Timeout al esperar la respuesta del robot.";
//        return posiciones;
//    }
//
//    QByteArray respuesta = m_serial.readAll();
//    while (m_serial.waitForReadyRead(100)) {
//        respuesta += m_serial.readAll();
//    }
//
//    qDebug() << "Respuesta del robot:" << respuesta;
//
//    // Procesar la respuesta
//    QString respStr = QString::fromLatin1(respuesta).trimmed();
//
//    if (respStr.startsWith("#p") && respStr.endsWith("*")) {
//        respStr = respStr.mid(2, respStr.length() - 3);  // Eliminar "#p" y "*"
//        QStringList partes = respStr.split("-", Qt::SkipEmptyParts);
//
//        for (int i = 0; i < partes.size() && i < 6; ++i) {
//            posiciones[i] = partes[i].toInt();
//        }
//    }
//    else {
//        qDebug() << "Formato de respuesta no válido.";
//    }
//
//    return posiciones;
//}
