/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** 主窗口
 ******************************************************************************
 *
 *  主窗口
 *
 *  @file       main_window.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "main_window.h"

#include <string>

#include "QMessageBox"
#include "communication/shared_data.h"
#include "ui_main_window.h"
#include "utils/log.h"

namespace phoenix {
namespace hmi {

MainWindow::MainWindow(int argc, char **argv, const std::string &work_space,
                       QWidget *parent)
    : QMainWindow(parent), work_space_(work_space), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // Set main window icon
  setWindowIcon(QIcon(":/images/icon.png"));

  // Adjust main window position
  // QRect rect;
  // rect.setX(100);
  // rect.setY(200);
  // rect.setSize(this->size());
  // this->setGeometry(rect);

  // Create map window
  widget_map_ = new WidgetMap();
  ui->mapgridLayout->addWidget(widget_map_, 0, 0);
  // ui->gridLayoutMaps->addWidget(widget_map_, 0, 0);

  //实例化一个QChart对象
  // QChart *chart = new QChart;
  // chart->setTitle("GDP增长图"); //设置标题
  // ui->trajectoryView->setChart(chart);  //将QChart对象设置到QChartView图表上

  // //设置坐标轴
  // QValueAxis* axisX = new QValueAxis;
  // axisX->setTitleText("年份"); //设置标题
  // axisX->setRange(1960, 2020); //设置范围
  // axisX->setTickCount(7);      //设置主刻度个数
  // axisX->setLineVisible(true); //设置轴线和刻度线可见性
  // axisX->setGridLineVisible(false); //设置网格线可见性

  // QValueAxis* axisY = new QValueAxis;
  // axisY->setTitleText("GDP(万亿:美元)"); //设置标题
  // axisY->setRange(0, 20);       //设置范围
  // axisY->setTickCount(21);      //设置主刻度个数
  // axisY->setLineVisible(true); //设置轴线和刻度线可见性
  // axisY->setGridLineVisible(false); //设置网格线可见性

  // //设置序列1
  // QLineSeries *series = new QLineSeries;
  // series->setName("中国");    //设置序列名
  // series->setColor(QColor(255,0,0)); //设置序列颜色

  // //添加数据点到序列
  // series->append(1960, 0.06);
  // series->append(1965, 0.07);
  // series->append(1970, 0.09);
  // series->append(1975, 0.16);
  // series->append(1980, 0.19);
  // series->append(1985, 0.30);
  // series->append(1990, 0.36);
  // series->append(1995, 0.73);
  // series->append(2000, 1.21);
  // series->append(2005, 2.29);
  // series->append(2010, 6.09);
  // series->append(2015, 11.06);
  // series->append(2020, 14.72);

  // //设置序列2
  // QLineSeries *series2 = new QLineSeries;
  // series2->setName("美国");    //设置序列名称
  // series2->setColor(QColor(0,0,255)); //设置序列颜色

  // //添加数据点到序列
  // series2->append(1960, 0.54);
  // series2->append(1965, 0.74);
  // series2->append(1970, 1.07);
  // series2->append(1975, 1.68);
  // series2->append(1980, 2.85);
  // series2->append(1985, 4.34);
  // series2->append(1990, 5.96);
  // series2->append(1995, 7.64);
  // series2->append(2000, 10.25);
  // series2->append(2005, 13.04);
  // series2->append(2010, 15.00);
  // series2->append(2015, 18.24);
  // series2->append(2020, 20.93);

  // //设置序列3
  // QLineSeries *series3 = new QLineSeries;
  // series3->setName("印度");    //设置序列名称
  // series3->setColor(QColor(128,128,128)); //设置序列颜色

  // //添加数据点到序列
  // series3->append(1960, 0.03);
  // series3->append(1965, 0.06);
  // series3->append(1970, 0.06);
  // series3->append(1975, 0.10);
  // series3->append(1980, 0.18);
  // series3->append(1985, 0.23);
  // series3->append(1990, 0.32);
  // series3->append(1995, 0.36);
  // series3->append(2000, 0.47);
  // series3->append(2005, 0.82);
  // series3->append(2010, 1.67);
  // series3->append(2015, 2.10);
  // series3->append(2020, 2.62);

  // //设置序列4
  // QLineSeries *series4 = new QLineSeries;
  // series4->setName("日本");    //设置序列名称
  // series4->setColor(QColor(144,238,144)); //设置序列颜色

  // //添加数据点到序列
  // series4->append(1960, 0.04);
  // series4->append(1965, 0.09);
  // series4->append(1970, 0.21);
  // series4->append(1975, 0.52);
  // series4->append(1980, 1.10);
  // series4->append(1985, 1.40);
  // series4->append(1990, 3.13);
  // series4->append(1995, 5.55);
  // series4->append(2000, 4.97);
  // series4->append(2005, 4.83);
  // series4->append(2010, 5.76);
  // series4->append(2015, 4.44);
  // series4->append(2020, 5.06);

  // //给Qchart添加序列
  // chart->addSeries(series);
  // chart->addSeries(series2);
  // chart->addSeries(series3);
  // chart->addSeries(series4);

  // //把序列设置到坐标轴
  // chart->setAxisX(axisX, series);
  // chart->setAxisY(axisY, series);
  // chart->setAxisX(axisX, series2);
  // chart->setAxisY(axisY, series2);
  // chart->setAxisX(axisX, series3);
  // chart->setAxisY(axisY, series3);
  // chart->setAxisX(axisX, series4);
  // chart->setAxisY(axisY, series4);

  Char_t str_buff[256] = {0};
  label_status_ = new QLabel();
  ui->statusbar->addWidget(label_status_);
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: MPSK");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: BDStar");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: Intertial Lab");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_F9K)
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: F9K");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_ZHD_RTK)
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: ZHD");
  label_status_->setText(str_buff);
#else
  com_snprintf(str_buff, sizeof(str_buff) - 1, "DEV_IMU: Undefined");
  label_status_->setText(str_buff);
#endif

  refresh_timer_ = this->startTimer(100);
  if (0 == refresh_timer_) {
    LOG_ERR << "MainWindow, failed to start refresh Timer";
  }
}

MainWindow::~MainWindow() {
  delete ui;
  LOG_INFO(3) << "MainWindow Deconstructed !";
}

void MainWindow::OpenWidgetSettingsPrefrence() {}

void MainWindow::OpenWidgetVehicleParameters() {}

void MainWindow::OpenWidgetDebugInfo() {}

void MainWindow::OpenWidgetAbout() {}

void MainWindow::timerEvent(QTimerEvent *event) {
  if (event->timerId() == refresh_timer_) {
    UpdateModuleInfo();
    UpdateStatusBar();

    widget_map_->Update();
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  LOG_INFO(3) << "Received close event!";
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  QMainWindow::mousePressEvent(event);
}

void MainWindow::moveEvent(QMoveEvent *event) { QMainWindow::moveEvent(event); }

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
}

void MainWindow::UpdateModuleInfo() {
  phoenix::framework::SharedData *shared_data =
      phoenix::framework::SharedData::instance();
}

void MainWindow::UpdateStatusBar() {
  char str_buff[256] = {0};
  QString background_normal = "background-color: rgb(82, 235, 233);";
  QString background_warn = "background-color: rgb(255, 255, 0);";
  QString background_err = "background-color: rgb(255, 0, 0);";
  QString background_timeout = "background-color: rgb(255, 128, 128);";
}

}  // namespace hmi
}  // namespace phoenix
