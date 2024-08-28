#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QDateTime>
#include <QPainter>
#include <QProcess>
#include <QThread>
#include <QDebug>
#include <QTimer>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstddef>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pullStream.h"
#include "connect.h"
#include "pcdecoder.h"
#include <QVTKOpenGLNativeWidget.h>

QRectF get_rect(int img_width, int img_height, float bbox[4]);

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
 
public:
    MainWindow(QWidget *parent = nullptr);
    static void customMessageHandler(QtMsgType type, 
                          const QMessageLogContext &context,
                          const QString &msg);
    void onSendPixmap(const QPixmap & pixmap);
    void refresh();
    void windowRefresh();
    void Connect();
    void clearOutput();
    void onPointcloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr  pc);
    ~MainWindow();

signals:
    void threadStartWork();
    void threadStopWork();
    void connectorStartWork();
    void connectorStopWork();
    void pcdecoderStartWork();
    void pcdecoderStopWork();
    void beginConnect(const QString&);

private:
    Ui::MainWindow *ui;
    void threadRun();
    void stopThread();
    void onWorkered();
    int res_cnt = -1;
    QTimer* refreshTimer;
    pullStreamWorker* psworker;
    Connector* connector;
    PointCloudDecoder* pcdecoder;
    QThread connectThread;
    QThread pullStreamThread;
    QThread pointcloudThread;
    static MainWindow* mainwindow;
};
#endif // MAINWINDOW_H
