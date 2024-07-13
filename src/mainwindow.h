
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QDateTime>
#include <QPainter>
#include <QProcess>
#include <QThread>
#include <QDebug>
#include <QTimer>
#include "pullStream.h"
#include "connect.h"

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
    ~MainWindow();

signals:
    void threadStartWork();
    void threadStopWork();
    void connectorStartWork();
    void connectorStopWork();
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
    QThread connectThread;
    QThread pullStreamThread;
    static MainWindow* mainwindow;
};
#endif // MAINWINDOW_H
