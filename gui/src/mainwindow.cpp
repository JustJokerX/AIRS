#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    de(new DEWidget),
    m_fullScreen(false)
{
    ui->setupUi(this);
    ui->mainLayout->addWidget(de);
    this->setFocus(); // what the fuck, why should i add this?!
}

MainWindow::~MainWindow()
{
    delete de;
    delete ui;
}


void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
        case Qt::Key_F1:                                    //F1为全屏和普通屏的切换键
            m_fullScreen = !m_fullScreen;
            if (m_fullScreen)
            {
                showFullScreen();
            }
            else
            {
                showNormal();
            }
           break;
        case Qt::Key_Escape:                                //ESC为退出键
            close();
            break;
    }
    // propgation
    de->keyPressEvent(event);
}
