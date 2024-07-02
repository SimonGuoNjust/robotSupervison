#ifndef PULL_STREAM_H
#define PULL_STREAM_H

#include <QObject>
#include <QString>
#include <QPixmap>
#include <QMutex>

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
    QString videoPath;
    QPixmap pixmap;
    workerStateType workerState{workerStateType::noPlay};
    QMutex mutex;
 
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
