#include "pullStream.h"

cv::Rect get_rect(int img_width, int img_height, float bbox[4]) {
    float l, r, t, b;
    float r_w = 640 / (img_width * 1.0);
    float r_h = 640 / (img_height * 1.0);

    if (r_h > r_w) {
        l = bbox[0];
        r = bbox[2];
        t = bbox[1] - (640 - r_w * img_height) / 2;
        b = bbox[3] - (640 - r_w * img_height) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - (640 - r_h * img_width) / 2;
        r = bbox[2] - (640 - r_h * img_width) / 2;
        t = bbox[1];
        b = bbox[3];
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    l = std::max(0.0f, l);
    t = std::max(0.0f, t);
    int width = std::max(0, std::min(int(round(r - l)), img_width - int(round(l))));
    int height = std::max(0, std::min(int(round(b - t)), img_height - int(round(t))));

    return cv::Rect(int(round(l)), int(round(t)), width, height);
}

void pullStreamWorker::drawResults(int img_width, int img_height, cv::Mat& img )
{ 
    QSharedMemory m_sharedMemory;
    m_sharedMemory.setKey("msg_recv");
    if(!m_sharedMemory.attach()){
    //    qDebug()<<"共享内存失败";
        return;
    }
    // qDebug() << "hg";
    m_sharedMemory.lock();
    char type;
    const char* from = (const char*)m_sharedMemory.data();
    memcpy(&type, from, 1);
    if (type != 4) return;
    int res_cnt = *(int*)(from + 1);
    if (res_cnt <= 0) return;
    memcpy(resultsBuff, from + 5, res_cnt * 4 * 7);
    m_sharedMemory.unlock();
    m_sharedMemory.detach();

    float* results = (float*)resultsBuff;
    float bbox[4];
    for (int i = 0; i < res_cnt; i++) {
        int basic_pos = i * 7;
        int keep_flag = results[basic_pos + 6];
        if (keep_flag == 1) {
            bbox[0] = results[basic_pos + 0];
            bbox[1] = results[basic_pos + 1];
            bbox[2] = results[basic_pos + 2];
            bbox[3] = results[basic_pos + 3];
            float conf = results[basic_pos + 4];
            cv::Rect r = get_rect(img.cols, img.rows, bbox);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, "apple", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            cv::putText(img, std::to_string(conf), cv::Point(r.x + 5, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
    }  
}

int pullStreamWorker::runningWorker = 0;
pullStreamWorker::pullStreamWorker()
    :QObject()
{
}
 
pullStreamWorker::~pullStreamWorker()
{
    clear();
}
 
void pullStreamWorker::doWork()
{
    ++runningWorker;
    if(videoPath.isEmpty())
    {
        workerState = workerStateType::videoPathIsEmpty;
        isRunning = false;
        clear();
        return;
    }
    isRunning = true;
 
    avformat_network_init();//初始化网络
 
    //设置传输协议为TCP协议
    av_dict_set(&options, "rtsp_transport", "udp", 0);
 
    // 设置TCP连接最大延时时间
    av_dict_set(&options, "max_delay", "100", 0);
 
    // 设置“buffer_size”缓存容量
    av_dict_set(&options, "buffer_size", "15000000", 0);
 
    // 设置avformat_open_input超时时间为3秒
    av_dict_set(&options, "stimeout", "3000000", 0);
 
    //打开视频文件
    auto result = avformat_open_input(&pFormatCtx,qPrintable(videoPath),nullptr,&options);
    if(result != 0)
    {
        workerState = workerStateType::openStreamFailed;
        qDebug()<<"打开流失败"<<result;
        isRunning = false;
        clear();
        return;
    }
 
    //获取多媒体流的信息（视频文件信息），一个视频文件中可能会同时包括视频文件、音频文件、字幕文件等多个媒体流。
    if(avformat_find_stream_info(pFormatCtx,nullptr) != 0)
    {
        qDebug()<<"获取视频文件信息失败";
        workerState = workerStateType::readStreamFailed;
        isRunning = false;
        clear();
        return;
    }
 
    //从多个媒体流中找到视频流
    int videoindex = av_find_best_stream(pFormatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if(videoindex == -1)
    {
        qDebug()<<"未发现视频流";
        workerState = workerStateType::noFildVideoStream;
        isRunning = false;
        clear();
        return;
    }
 
    auto pCodecParameter = pFormatCtx->streams[videoindex]->codecpar;
    auto pCodec = avcodec_find_decoder_by_name("h264_cuvid");
    
    if(pCodec == nullptr)
    {
        workerState = workerStateType::noFildCodec;
        qDebug()<<"未找到编解码器";
        isRunning = false;
        clear();
        return;
    }
 
    pCodecCtx = avcodec_alloc_context3(pCodec);//初始化一个编解码上下文
    av_opt_set(pCodecCtx->priv_data, "tune", "zerolatency", 0);
    pCodecCtx->pkt_timebase.num = pFormatCtx->streams[videoindex]->time_base.num; // 我的是加了这里才正常
    pCodecCtx->pkt_timebase.den = pFormatCtx->streams[videoindex]->time_base.den; // 我的是加了这里才正常
    //pCodecParameter中的流参数复制到pCodecCtx
    avcodec_parameters_to_context(pCodecCtx,pCodecParameter);
 
    if(avcodec_open2(pCodecCtx, pCodec,nullptr) < 0)   //打开解码器，使用pCodec初始化pCodecCtx
    {
        workerState = workerStateType::openCodecFailed;
        qDebug()<<"未打开编解码器";
        isRunning = false;
        clear();
        return;
    }
 
    pFrame = av_frame_alloc();//存放从AVPacket中解码出来的原始数据
    pFrameRGB = av_frame_alloc();//存放原始数据转换的目标数据
 
    packet = av_packet_alloc();
    av_new_packet(packet, pCodecCtx->width * pCodecCtx->height);//分配packet的有效载荷并初始化其字段
 
    //av_image_get_buffer_size：返回存储给定参数的图像数据所需数据量的大小（以字节为单位）
    //av_malloc：分配适合所有内存访问的对齐方式的内存块
    auto buf = static_cast<uchar *>(av_malloc(static_cast<size_t>(av_image_get_buffer_size(AV_PIX_FMT_RGB32,
                                                                                           pCodecCtx->width,
                                                                                           pCodecCtx->height, 1))));
 
    //根据后5个参数的内容填充前两个参数，成功返回源图像的大小，失败返回一个负值
    if(av_image_fill_arrays(pFrameRGB->data,   // 需要填充的图像数据指针
                         pFrameRGB->linesize,
                         buf,
                         AV_PIX_FMT_RGB32, //图像的格式
                         pCodecCtx->width,
                         pCodecCtx->height,
                         1) < 0)    //图像数据中linesize的对齐
    {
        workerState = workerStateType::getStreamImageFailed;
        qDebug()<<"获取流图像失败";
        isRunning = false;
        clear();
        return;
    }
 
    //用于视频图像的转换，将源数据转换为目标数据
    
    img_convert_ctx = sws_getContext(pCodecCtx->width,
                                    pCodecCtx->height,
                                    pCodecCtx->pix_fmt,
                                    pCodecCtx->width,
                                    pCodecCtx->height,
                                    AV_PIX_FMT_RGB32,
                                    SWS_BICUBIC,
                                    nullptr,
                                    nullptr,
                                    nullptr);
 
    workerState = workerStateType::playing;
    while(av_read_frame(pFormatCtx, packet) >= 0)
    {
        if(!isRunning)
        {
            break;
        }
 
        if(packet->stream_index == videoindex)//此流是视频流
        {
            //解码一帧视频数据
            //提供原始数据包数据作为解码器的输入
             if(avcodec_send_packet(pCodecCtx, packet) != 0)
             {
                 qDebug()<<"输入待解码的数据出错";
                 continue;
             }
 
             if(avcodec_receive_frame(pCodecCtx, pFrame) != 0)
             {  
                qDebug() << avcodec_receive_frame(pCodecCtx, pFrame);
                 qDebug()<<"从解码器返回解码后的输出数据出错";
                 continue;
             }
            // qDebug()<<"frame " << pFrame->pts << " arrive" << endl;
             //此函数可以：1.图像色彩空间转换；2.分辨率缩放；3.前后图像滤波处理。
             sws_scale(img_convert_ctx,
                       static_cast<const uchar* const*>(pFrame->data),
                       pFrame->linesize,
                       0,
                       pCodecCtx->height,
                       pFrameRGB->data,
                       pFrameRGB->linesize);
            // img.data = static_cast<uchar*>(pFrameRGB->data[0]);
                 
            // cv::Mat img(cv::Size(pCodecCtx->width, pCodecCtx->height), CV_8UC3, static_cast<uchar*>(pFrameRGB->data[0]));
            // drawResults(pCodecCtx->width, pCodecCtx->height, img);
             pixmap = QPixmap::fromImage(QImage(static_cast<uchar*>(pFrameRGB->data[0]),
                                                pCodecCtx->width,
                                                pCodecCtx->height,
                                                QImage::Format_RGB32));
            emit sendPixmap(pixmap);
 
             QThread::msleep(5);
        }
    }
 
    clear();
}
 
void pullStreamWorker::stopWork()
{
    QMutexLocker locker(&mutex);
    isRunning = false;
    workerState = workerStateType::noPlay;
    qDebug()<<"停止工作";
}
 
void pullStreamWorker::clear()
{
    if(img_convert_ctx)
        sws_freeContext(img_convert_ctx);
    if(pFrameRGB)
        av_frame_free(&pFrameRGB);
    if(pFrame)
        av_frame_free(&pFrame);
    if(pCodecCtx)
        avcodec_free_context(&pCodecCtx);
    if(pFormatCtx)
        avformat_close_input(&pFormatCtx);
    if(packet)
        av_packet_free(&packet);
    if(options)
        av_dict_free(&options);
 
    img_convert_ctx = nullptr;
    pFrameRGB = nullptr;
    pFrame = nullptr;
    pCodecCtx = nullptr;
    pFormatCtx = nullptr;
    packet = nullptr;
    options = nullptr;
    pixmap = QPixmap();
 
    emit finished();
    --runningWorker;
}
 
const QPixmap * pullStreamWorker::getPixmap() const
{
    return &pixmap;
}
 
QString pullStreamWorker::getStateString()
{
    switch (workerState)
    {
        case workerStateType::noPlay:return tr("未播放");
        case workerStateType::playing:return tr("正在播放");
        case workerStateType::noFildCodec:return tr("未找到编解码器");
        case workerStateType::openCodecFailed:return tr("未打开编解码器");
        case workerStateType::openStreamFailed:return tr("打开流失败");
        case workerStateType::readStreamFailed:return tr("获取视频流信息失败");
        case workerStateType::videoPathIsEmpty:return tr("未设置视频地址");
        case workerStateType::noFildVideoStream:return tr("未发现视频流");
        case workerStateType::getStreamImageFailed:return tr("获取流图像失败，可能是多处在获取同一视频流");
    }
}
 
void pullStreamWorker::setVideoPath(const QString &value)
{
    videoPath = value;
}
 
bool pullStreamWorker::getIsRunning() const
{
    return isRunning;
}
 
int pullStreamWorker::getRunningWorker()
{
    return runningWorker;
}
 
QString pullStreamWorker::getVideoPath() const
{
    return videoPath;
}