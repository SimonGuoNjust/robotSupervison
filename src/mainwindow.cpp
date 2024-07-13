#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
enum { max_length = 1024};
MainWindow* MainWindow::mainwindow = NULL;

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
    connect(ui->bt4, &QPushButton::clicked,this,&MainWindow::clearOutput);
    // connect(this->refreshTimer, &QTimer::timeout, this, &MainWindow::refresh);
    connect(connector, &Connector::sig_newDevice, this, &MainWindow::refresh);
    connect(this->refreshTimer, &QTimer::timeout, this, &MainWindow::windowRefresh);

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

    QProcess *process = new QProcess();
    process->start("D:\\mediamtx\\mediamtx.exe");
    connect(process, &QProcess::readyRead, [=](){
        QByteArray procOutput = process->readAll();
        qDebug()<<"[mediamtx]"<< procOutput.data();
    });

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
    // qDebug() << "refresh";

    // if (res_cnt > 0)
    // {
        // QImage qImage = pixmap.toImage();
    //     QPainter painter;
    //     painter.begin(&qImage);
    //     painter.setPen(QPen(Qt::red));
    //     float* results = (float*)resultsBuff;
    //     float bbox[4];
    //     for (int i = 0; i < res_cnt; i++) {
    //         int basic_pos = i * 7;
    //         int keep_flag = results[basic_pos + 6];
    //         if (keep_flag == 1) {
    //             bbox[0] = results[basic_pos + 0];
    //             bbox[1] = results[basic_pos + 1];
    //             bbox[2] = results[basic_pos + 2];
    //             bbox[3] = results[basic_pos + 3];
    //             // qDebug() << c_x << c_y << w << h;
    //             QRectF rect = get_rect(ui->label->width(), ui->label->height(), bbox);
    //             painter.drawRect(rect);
    //             float conf = results[basic_pos + 4];
    //             painter.drawText(rect.x(), rect.y(), QString::fromStdString(std::to_string(conf)));
    //             painter.drawText(rect.x(), rect.y() - 10, tr("apple"));
    //     }

    //     }
    //     painter.end();
    //     ui->label->setPixmap(QPixmap::fromImage(qImage).scaled(ui->label->size()));
    // }
    // else
    // {
        ui->label->setPixmap(pixmap.scaled(ui->label->size()));
    // }
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
