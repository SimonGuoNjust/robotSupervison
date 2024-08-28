#ifndef PC_DECODER_H
#define PC_DECODER_H
#include <boost/asio/ip/tcp.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include<boost/make_shared.hpp>
#include <QObject>
#include <QMutex>
#include <QDebug>
#include <QThread>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h> 
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <qobject.h>
#include <qobjectdefs.h>
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
extern QMutex pc_m;
class PointCloudDecoder : public QObject
{
    Q_OBJECT
public:
    PointCloudDecoder();
    ~PointCloudDecoder();
    void doWork();
    void stopWork();
    void clear();
    bool getIsRunning() const;
    static int getRunningWorker();
    void onTcpNotify(boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp_);
signals:
    void finished();
    void pcUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr);

private:
    bool tcpConnected = false;
    unsigned short targetPortUdp = 27777;
    bool isRunning{false};
    QMutex mutex;
    static int runningWorker;
    boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp = nullptr;
};

// class PointCloudDecoder: public QObject
// {
//     Q_OBJECT
// public:
//     PointCloudDecoder();
//     ~PointCloudDecoder();
//     void doWork();
//     void stopWork();
//     void clear();
//     bool getIsRunning() const;
//     static int getRunningWorker();
//     void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_);
    
// public slots:
//     void onTcpNotify(boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp_);
 
// signals:
//     void finished();
//     void pcUpdate();

// private:
//     bool tcpConnected = false;
//     boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp = nullptr;
//     std::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> pc_compressor;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
//     unsigned short targetPortUdp = 27777;
//     bool isRunning{false};
//     QMutex mutex;
//     static int runningWorker;
// };

#endif