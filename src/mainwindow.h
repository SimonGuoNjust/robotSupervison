
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QThread>
#include <QDebug>
#include <QTimer>
#include "pullStream.h"
#include "connect.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
 
public:
    MainWindow(QWidget *parent = nullptr);
    void onSendPixmap(const QPixmap & pixmap);
    void refresh();
    void Connect();
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
    QTimer* refreshTimer;
    pullStreamWorker* psworker;
    Connector* connector;
    QThread connectThread;
    QThread pullStreamThread;
};
#endif // MAINWINDOW_H
