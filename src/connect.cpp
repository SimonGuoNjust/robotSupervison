#include "connect.h"
#include <iostream>
#include <QSharedMemory>
using boost::asio::ip::udp;
 
int Connector::runningWorker = 0;
Connector::Connector()
    :QObject(), sUdp(this->ios, udp::endpoint(udp::v4(), 0))
{

}
 
Connector::~Connector()
{
    clear();
}

void Connector::recConnectMsg (udp::endpoint* rc_endpoint, const boost::system::error_code& error, std::size_t )
{   
    if (!error)
    {   
        // connectorState = connectStateType::waitingTcp;
        connect_msg* c_msg = (connect_msg*)connectMsgBuffer;
        
        if (c_msg->status == MSEARCH)
        {
            const char* detected_addr = (rc_endpoint->address()).to_string().c_str();
            memcpy(c_msg->ip_addr, detected_addr, std::strlen(detected_addr));
            c_msg->ip_addr[std::strlen(detected_addr)] = 0;
            c_msg->ip_len = std::strlen(detected_addr);
            c_msg->udp_port = targetPortUdp;
            if (devices.addDevice(*c_msg) == 0)
            {
                m_sharedMemory->lock();
                char* to = (char*)m_sharedMemory->data();
                memcpy(to, connectMsgBuffer, 102);
                m_sharedMemory->unlock(); 
            emit sig_newDevice();
            }
        }
        else if (c_msg->status == MREADY)
        {
            
            connectorState = connectStateType::running;
        }
        // devices.printDevices();
        
    }
}

void Connector::doWork()
{  
    QSharedMemory shareMemory;
    shareMemory.setKey("msg_recv");
    if (shareMemory.isAttached())
    {
        if (!shareMemory.detach())
        {
            qDebug() << shareMemory.errorString();
            return;
        }
    }
    if (!shareMemory.create(1024))
    {
        qDebug() << shareMemory.errorString();
        return;
    }
    m_sharedMemory = &shareMemory;

    sUdp = boost::asio::ip::udp::socket(ios, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));
    sUdp.set_option(boost::asio::socket_base::receive_buffer_size(256));
    sUdp.set_option(boost::asio::socket_base::reuse_address(true));
    sUdp.set_option(boost::asio::socket_base::broadcast(true));

    ++runningWorker;
    isRunning = true;
    auto rc_endpoint = boost::asio::ip::udp::endpoint();
    auto bc_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), targetPortUdp);

    char user_name[20] = "admin";
    char status = MSEARCH;
    char ip_addr[16] = "192.168.137.1";
    unsigned short tcp_port = 27777;
    int ec;
    // ios.run();
    while(1)
    {
        if(!isRunning)
        {
            break;
        }
        QThread::msleep(300);
        if (connectorState == connectStateType::inSearch)
        {
            connect_msg msg = connect_msg(user_name, status, ip_addr, std::strlen(ip_addr), tcp_port, "");
            ec = sUdp.send_to(boost::asio::buffer(msg.to_string(), 102), bc_endpoint);
            if (ec < 0)
            {
                qDebug() << "send error" << ec;
            }
            
        }
        else if (connectorState == connectStateType::connecting)
        {
            status = MREADY;
            auto to_ep = devices.getConnectedDevice();
            connect_msg msg = connect_msg(user_name, status, ip_addr, std::strlen(ip_addr), tcp_port, videoPath);;
            ec = sUdp.send_to(boost::asio::buffer(msg.to_string(), 102), to_ep);
            if (ec < 0)
            {
                qDebug() << "send error" << ec;
            }
        }
        sUdp.async_receive_from(boost::asio::buffer(connectMsgBuffer), rc_endpoint,
            boost::bind(&Connector::recConnectMsg, this, &rc_endpoint, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
        ios.poll_one();  
    }
    clear();
        
}

void Connector::connect(const QString &value)
{
    devices.setConnectedDeivce(value.toStdString());
    sUdp.set_option(boost::asio::socket_base::reuse_address(false));
    sUdp.set_option(boost::asio::socket_base::broadcast(false));
    connectorState = connectStateType::connecting;
}

void Connector::setVideoPath(const QString &value)
{
    const char* path = value.toStdString().c_str();
    memcpy(videoPath, path, std::strlen(path) + 1);
}

void Connector::stopWork()
{
    QMutexLocker locker(&mutex);
    isRunning = false;
    qDebug()<<"停止工作";
}
 
void Connector::clear()
{
    
    m_sharedMemory->detach();
    m_sharedMemory = nullptr;
    sUdp.cancel();
    sUdp.close();
    ios.stop();
    emit finished();
    --runningWorker;
}

bool Connector::getIsRunning() const
{
    return isRunning;
}
 
int Connector::getRunningWorker()
{
    return runningWorker;
}
