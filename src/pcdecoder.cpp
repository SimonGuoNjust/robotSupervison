#include <boost/smart_ptr/make_shared_array.hpp>
#include <pcdecoder.h>
 
int PointCloudDecoder::runningWorker = 0;
PointCloudDecoder::PointCloudDecoder() :QObject()
{
}

// void PointCloudDecoder::setPointCloud(_)
// {
//     pointcloud = pc_;
// }

PointCloudDecoder::~PointCloudDecoder()
{
    clear();
}

void PointCloudDecoder::onTcpNotify(boost::shared_ptr<boost::asio::ip::tcp::iostream> sTcp_)
{
    qDebug() << "received tcp notification";
    if (!tcpConnected)
    {
        qDebug() << "tcp connected";
        sTcp = sTcp_;
        tcpConnected = true;
    }
}

void PointCloudDecoder::doWork()
{  
    pcl::io::compression_Profiles_e compressionProfile;
    compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    const pcl::io::configurationProfile_t selectedProfile =
          pcl::io::compressionProfiles_[compressionProfile];
    bool showStatistics;
    double pointResolution;
    float octreeResolution;
    bool doVoxelGridDownDownSampling;
    unsigned int iFrameRate;
    bool doColorEncoding;
    unsigned int colorBitResolution;

    pointResolution = selectedProfile.pointResolution;
    octreeResolution = float(selectedProfile.octreeResolution);
    doVoxelGridDownDownSampling = false;
    iFrameRate = selectedProfile.iFrameRate;
    doColorEncoding = selectedProfile.doColorEncoding;
    colorBitResolution = selectedProfile.colorBitResolution;
    auto pc_compressor = boost::make_shared<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
        compressionProfile,
        showStatistics,
        pointResolution,
        octreeResolution,
        doVoxelGridDownDownSampling,
        iFrameRate,
        doColorEncoding,
        static_cast<unsigned char>(colorBitResolution)
    );

    ++runningWorker;
    isRunning = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    while(1)
    {
        QThread::msleep(20);
        if(!isRunning)
        {
            break;
        }
        
        if (!tcpConnected)
        {
            continue;
        }
        if(sTcp->fail())
        {
            qDebug() << "socket read fail";
            continue;
        }
        // qDebug() << "decode pointcloud";
        pc_m.lock();
        pc_compressor->decodePointCloud(*sTcp, pointcloud);
        pc_m.unlock();
        emit pcUpdate(pointcloud);
    }
    clear();       
}


void PointCloudDecoder::stopWork()
{
    QMutexLocker locker(&mutex);
    isRunning = false;
    qDebug()<<"停止工作";
}
 
void PointCloudDecoder::clear()
{
    emit finished();
    --runningWorker;
}

bool PointCloudDecoder::getIsRunning() const
{
    return isRunning;
}
 
int PointCloudDecoder::getRunningWorker()
{
    return runningWorker;
}
