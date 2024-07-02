#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
enum { max_length = 1024};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->refreshTimer = new QTimer(this);
    this->refreshTimer->start(30);
    this->connector = new Connector();
    this->connector->moveToThread(&connectThread);
    this->psworker = new pullStreamWorker();
    ui->stremPath_label->setText("rtsp://localhost:8554/stream");
    QString streamPath = ui->stremPath_label->toPlainText();
    this->psworker->setVideoPath(streamPath);
    this->psworker->moveToThread(&pullStreamThread);
    connect(this,&MainWindow::threadStartWork,psworker,&pullStreamWorker::doWork);
    connect(this,&MainWindow::connectorStartWork,connector,&Connector::doWork);
    connect(this,&MainWindow::threadStopWork,psworker,&pullStreamWorker::stopWork, Qt::DirectConnection);
    connect(this,&MainWindow::connectorStopWork,connector,&Connector::stopWork, Qt::DirectConnection);
    connect(psworker, &pullStreamWorker::sendPixmap,this,&MainWindow::onSendPixmap);
    connect(psworker, &pullStreamWorker::finished, this, &MainWindow::onWorkered);
    connect(connector, &Connector::finished, this, &MainWindow::onWorkered);
    connect(this, &MainWindow::beginConnect, connector, &Connector::connect, Qt::DirectConnection);
    connect(ui->bt1,&QPushButton::clicked,this,&MainWindow::threadRun);
    connect(ui->bt2, &QPushButton::clicked,this,&MainWindow::stopThread);
    connect(ui->bt0, &QPushButton::clicked,this,&MainWindow::Connect);
    // connect(this->refreshTimer, &QTimer::timeout, this, &MainWindow::refresh);
    connect(connector, &Connector::sig_newDevice, this, &MainWindow::refresh);
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
    char to[50];
    const char* from = (const char*)m_sharedMemory.data();
    memcpy(to, from, 39);
    m_sharedMemory.unlock();
    m_sharedMemory.detach();
    connect_msg* msg_ = (connect_msg*)to;
    // ui->statusbar->showMessage(msg_->ip_addr, 1000);
    ui->comboBox->addItem(msg_->senderName);
    ui->bt0->setEnabled(true);
    ui->bt1->setEnabled(true);
    ui->bt2->setEnabled(true);
}

void MainWindow::onSendPixmap(const QPixmap &pixmap)
{   
    // qDebug() << "refresh";
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
}


MainWindow::~MainWindow()
{
    emit threadStopWork();
    emit connectorStopWork();
 
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
    psworker->deleteLater();
    connector->deleteLater();

    delete ui;
}
