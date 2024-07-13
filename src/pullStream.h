#ifndef PULL_STREAM_H
#define PULL_STREAM_H
#include <string>
#include <QObject>
#include <QString>
#include <QPixmap>
#include <QMutex>
#include <QDebug>
#include <QThread>
#include <QSharedMemory>
#include <opencv2/opencv.hpp>
 
extern "C"
{
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavdevice/avdevice.h>
    #include <libavformat/version.h>
    #include <libavutil/time.h>
    #include <libavutil/mathematics.h>
    #include <libavutil/imgutils.h>
};

class pullStreamWorker: public QObject
{
    Q_OBJECT
public:
    pullStreamWorker();
    ~pullStreamWorker();
    void doWork();
    void stopWork();
    void clear();
 
    const QPixmap *getPixmap() const;
    QString getStateString();
 
    void setVideoPath(const QString &value);
 
    bool getIsRunning() const;
 
    static int getRunningWorker();
 
    QString getVideoPath() const;
 
signals:
    void sendPixmap(const QPixmap &);
    void finished();
 
private:
    enum class workerStateType
    {
        noPlay,
        playing,
        getStreamImageFailed,
        videoPathIsEmpty,
        openStreamFailed,
        readStreamFailed,
        noFildVideoStream,
        noFildCodec,
        openCodecFailed
    };
 
    bool isRunning{false};
    void drawResults(int img_width, int img_height, cv::Mat& rgbFrame);
    QString videoPath;
    QPixmap pixmap;
    workerStateType workerState{workerStateType::noPlay};
    QMutex mutex;
    char resultsBuff[4000];
 
    struct AVFormatContext * pFormatCtx{nullptr};
    struct AVCodecContext * pCodecCtx{nullptr};
    struct SwsContext *img_convert_ctx{nullptr};
    struct AVFrame * pFrame{nullptr};
    struct AVFrame * pFrameRGB{nullptr};
    struct AVPacket * packet{nullptr};
    struct AVDictionary *options{nullptr};
 
    static int runningWorker;
};


#endif
