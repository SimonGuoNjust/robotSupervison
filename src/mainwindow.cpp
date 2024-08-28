#include "mainwindow.h"
#include "connect.h"
#include "pointcloud.h"
#include "ui_mainwindow.h"
#include <QVTKOpenGLNativeWidget.h>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <pcl/impl/point_types.hpp>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointerBase.h>
enum { max_length = 1024};
MainWindow* MainWindow::mainwindow = NULL;
QMutex pc_m;
void MainWindow::customMessageHandler(QtMsgType type, 
                          const QMessageLogContext &context,
                          const QString &msg)
{
    //Q_UNUSED(context)
    QDateTime _datetime = QDateTime::currentDateTime();
    QString szDate = _datetime.toString("yyyy-MM-dd hh:mm:ss.zzz");//"yyyy-MM-dd hh:mm:ss ddd"
    QString txt(szDate);

    switch (type)
    {
        case QtDebugMsg://调试信息提示
        {
            txt += QString(" [Debug] ");
            break;
        }
        case QtInfoMsg://信息输出
        {
            txt += QString(" [Info] ");
            break;
        }
        case QtWarningMsg://一般的warning提示
        {
            txt += QString(" [Warning] ");
            break;
        }
        case QtCriticalMsg://严重错误提示
        {
            txt += QString(" [Critical] ");
            break;
        }
        case QtFatalMsg://致命错误提示
        {
            txt += QString(" [Fatal] ");
            //abort();
            break;
        }
        default:
        {
            txt += QString(" [Trace] ");
            break;
        }
    }
    txt.append(msg);
    txt.append(QString("\n"));
    if(MainWindow::mainwindow != NULL){
        QString nowTxt = MainWindow::mainwindow->ui->debugInfo->text();
        MainWindow::mainwindow->ui->debugInfo->setText(nowTxt+=txt);
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{   
    ui->setupUi(this);
    MainWindow::mainwindow = this;
    // qInstallMessageHandler(customMessageHandler);
    this->refreshTimer = new QTimer(this);
    this->refreshTimer->start(30);
    this->connector = new Connector();
    this->connector->moveToThread(&connectThread);
    this->psworker = new pullStreamWorker();
    ui->stremPath_label->setText("rtsp://localhost:8554/stream");
    QString streamPath = ui->stremPath_label->toPlainText();
    this->psworker->setVideoPath(streamPath);
    this->psworker->moveToThread(&pullStreamThread);
    this->pcdecoder = new PointCloudDecoder();
    this->pcdecoder->moveToThread(&pointcloudThread);

    qRegisterMetaType<boost::shared_ptr<boost::asio::ip::tcp::iostream>>("boost::shared_ptr<boost::asio::ip::tcp::iostream>");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    connect(this,&MainWindow::threadStartWork,psworker,&pullStreamWorker::doWork);
    connect(this,&MainWindow::connectorStartWork,connector,&Connector::doWork);
    connect(this, &MainWindow::pcdecoderStartWork, pcdecoder, &PointCloudDecoder::doWork);
    connect(this,&MainWindow::threadStopWork,psworker,&pullStreamWorker::stopWork, Qt::DirectConnection);
    connect(this,&MainWindow::connectorStopWork,connector,&Connector::stopWork, Qt::DirectConnection);
    connect(this, &MainWindow::pcdecoderStopWork, pcdecoder, &PointCloudDecoder::stopWork);
    connect(psworker, &pullStreamWorker::sendPixmap,this,&MainWindow::onSendPixmap);
    connect(psworker, &pullStreamWorker::finished, this, &MainWindow::onWorkered);
    connect(connector, &Connector::finished, this, &MainWindow::onWorkered);
    connect(pcdecoder, &PointCloudDecoder::finished, this, &MainWindow::onWorkered);
    connect(this, &MainWindow::beginConnect, connector, &Connector::connect, Qt::DirectConnection);
    connect(ui->bt1,&QPushButton::clicked,this,&MainWindow::threadRun);
    connect(ui->bt2, &QPushButton::clicked,this,&MainWindow::stopThread);
    connect(ui->bt0, &QPushButton::clicked,this,&MainWindow::Connect);
    connect(ui->bt4, &QPushButton::clicked,this,&MainWindow::clearOutput);
    // connect(this->refreshTimer, &QTimer::timeout, this, &MainWindow::refresh);
    connect(connector, &Connector::sig_newDevice, this, &MainWindow::refresh);
    connect(this->refreshTimer, &QTimer::timeout, this, &MainWindow::windowRefresh);
    connect(connector, &Connector::sig_tcpNotify, pcdecoder, &PointCloudDecoder::onTcpNotify, Qt::DirectConnection);
    connect(pcdecoder, &PointCloudDecoder::pcUpdate, this, &MainWindow::onPointcloudUpdate, Qt::DirectConnection);

    QPalette p;
    p.setColor(QPalette::Background, Qt::black);
    p.setColor(QPalette::WindowText, Qt::white);
    ui->scrollArea->setAutoFillBackground(true);
    ui->scrollArea->setPalette(p);

    ui->bt0->setEnabled(false);
    ui->bt1->setEnabled(false);
    ui->bt2->setEnabled(false);
    ui->statusbar->showMessage(tr("Ready!"), 2000);
    if (!connectThread.isRunning())
    {
        qDebug() << "connecting";
        connectThread.start();
    }
    emit connectorStartWork();

    if (!pointcloudThread.isRunning())
    {
        qDebug() << "pointcloud starts";
        pointcloudThread.start();
    }
    emit pcdecoderStartWork();

    QProcess *process = new QProcess();
    process->start("D:\\mediamtx\\mediamtx.exe");
    connect(process, &QProcess::readyRead, [=](){
        QByteArray procOutput = process->readAll();
        qDebug()<<"[mediamtx]"<< procOutput.data();
    });

    //点云加载
    // pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // this->pcdecoder->setPointCloud(pointcloud);
    // pcl::io::loadPCDFile<pcl::PointXYZ>("D:/Files/dev/robotSupervison/src/rabbit.pcd", *pointcloud);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointcloud, red, green, blue);//自定义点云颜色
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    ui->pcvis->viewer->addPointCloud<pcl::PointXYZ>(pointcloud, "cloud");
    ui->pcvis->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud");//设置点云单个点的大小
    ui->pcvis->viewer->addCoordinateSystem(1);
    ui->pcvis->viewer->initCameraParameters();
    ui->pcvis->viewer->setCameraPosition(-0.83, -0.522, -0.91, 0.0912, -0.970, 0.226);
    // ui->pcvis->SetRenderWindow(viewer->getRenderWindow());
    // viewer->setupInteractor(ui->pcvis->GetInteractor(), ui->pcvis->GetRenderWindow());
    ui->pcvis->update();
}    

void MainWindow::onPointcloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
    // qDebug() << "update"; 
    pc_m.lock();
    ui->pcvis->viewer->updatePointCloud(pc, "cloud");
    ui->pcvis->update();
    pc_m.unlock();
}

void MainWindow::windowRefresh()
{
    this->ui->debugInfo->adjustSize();
}

void MainWindow::clearOutput()
{

}

void MainWindow::Connect()
{
    QString streamPath = ui->stremPath_label->toPlainText();
    this->psworker->setVideoPath(streamPath);
    this->connector->setVideoPath(streamPath);
    emit beginConnect(ui->comboBox->currentText());
}
void MainWindow::refresh()
{
    QSharedMemory m_sharedMemory;
    m_sharedMemory.setKey("msg_recv");
     if(!m_sharedMemory.attach()){
    //    qDebug()<<"共享内存失败";
        return;
    }
    m_sharedMemory.lock();
    char type;
    const char* from = (const char*)m_sharedMemory.data();

    memcpy(&type, from, 1);
    if (type == MSEARCH)
    {
        char to[102];
        memcpy(to, from + 1, 102);
        connect_msg* msg_ = (connect_msg*)to;
        ui->comboBox->addItem(msg_->senderName);
        ui->bt0->setEnabled(true);
        ui->bt1->setEnabled(true);
        ui->bt2->setEnabled(true);
    }
    m_sharedMemory.unlock();
    m_sharedMemory.detach();
}

void MainWindow::onSendPixmap(const QPixmap &pixmap)
{   
    ui->label->setPixmap(pixmap.scaled(ui->label->size()));
}

void MainWindow::threadRun()
{
    if (!pullStreamThread.isRunning())
    {
        QString streamPath = ui->stremPath_label->toPlainText();
        qDebug() << "pull streaming from " << streamPath;
        
        pullStreamThread.start();
    }
    emit threadStartWork();
}

void MainWindow::stopThread()
{
    emit threadStopWork();
}

void MainWindow::onWorkered()
{
    auto worker = sender();
    if(worker == psworker)
    {
        qDebug()<<"finished";
        pullStreamThread.quit();
        pullStreamThread.wait();
    }
    if (worker == connector)
    {
        qDebug()<<"connect finished";
        connectThread.quit();
        connectThread.wait();
    }
    if (worker == pcdecoder)
    {
        qDebug()<<"pointcloud finished";
        pointcloudThread.quit();
        pointcloudThread.wait();
    }
}


MainWindow::~MainWindow()
{
    emit threadStopWork();
    emit connectorStopWork();
    emit pcdecoderStopWork();
 
    if(pullStreamThread.isRunning())
    {
        pullStreamThread.quit();
        pullStreamThread.wait();
    }
    if(connectThread.isRunning())
    {
        connectThread.quit();
        connectThread.wait();
    }
    if(pointcloudThread.isRunning())
    {
        pointcloudThread.quit();
        pointcloudThread.wait();
    }
    psworker->deleteLater();
    connector->deleteLater();
    pcdecoder->deleteLater();

    delete ui;
}
