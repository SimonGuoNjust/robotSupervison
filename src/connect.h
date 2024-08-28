#ifndef CONNECT_H
#define CONNECT_H
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/bind.hpp>
#include "utils.h"
#include <QObject>
#include <QString>
#include <QBuffer>
#include <QMutex>
#include <QDebug>
#include <QThread>
#include <QSharedMemory>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstddef>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <pcl/point_cloud.h>

class Connector: public QObject
{
    Q_OBJECT
public:
    Connector();
    ~Connector();
    void doWork();
    void stopWork();
    void clear();

    void setVideoPath(const QString &value);
    void connect(const QString &value);
 
    bool getIsRunning() const;
 
    static int getRunningWorker();
 
signals:
    void finished();
    void sig_newDevice();
    void sig_tcpNotify(boost::shared_ptr<boost::asio::ip::tcp::iostream>);
 
private:
    void recConnectMsg(boost::asio::ip::udp::endpoint* rc_endpoint, const boost::system::error_code& error, std::size_t);
    enum class connectStateType
    {
        inSearch,
        connecting,
        running,
    };

    connectStateType connectorState{connectStateType::inSearch};
    int frameCnt = 0;
    char connectMsgBuffer[4096];
    char videoPath[60];
    boost::asio::io_service ios;
    boost::asio::ip::udp::socket sUdp;
    boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp = nullptr;
    unsigned short targetPortUdp = 27777;
    bool isRunning{false};
    QMutex mutex;
    DevicesList devices;
    static int runningWorker;
    QSharedMemory *m_sharedMemory;
};


#endif
