#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "deserializer.h"

#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QChartView>

#include <QtCharts/QtCharts>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    createActions();
    createMenus();

    chart = new QChart;
    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    legendFont.setPointSize(40);
    chart->legend()->setFont(legendFont);
    this->setCentralWidget(chartView);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete chart;
}

void MainWindow::readForcesDataSlot()
{
    Deserializer deserializer;
    auto [forces, lidar_data] = deserializer.deserializeData("/home/ihor/Simulation_gazebo/dataRos.json");
    _forces = std::move(forces);
    _lidar_data = std::move(lidar_data);

    qDebug() << "Read forces" << _forces.size() << " items.";
    if (_forces.size() > 0)
    {
        current_forces_chunk = 0;
        current_lidar_chunk = 0;
    }

    qDebug() << "Read lidar" << _lidar_data.size() << " items.";
    if (_lidar_data.size() > 0)
    {
        current_lidar_chunk = 0;
    }
}

void MainWindow::plotNextForcesChunkSlot()
{
    if (-1 == current_forces_chunk || _forces.size() == current_forces_chunk)
    {
        qWarning() << "no forces data left. ignoring...";
        return;
    }

    plotForcesData(current_forces_chunk);
    current_forces_chunk++;
}

void MainWindow::plotNextLidarChunkSlot()
{
    if (-1 == current_lidar_chunk || _lidar_data.size() == current_lidar_chunk)
    {
        qWarning() << "no lidar data left. ignoring...";
        return;
    }

    plotLidarData(current_lidar_chunk);
    current_lidar_chunk++;
}

void MainWindow::createMenus()
{
    _forcesMenu = menuBar()->addMenu(tr("Distance sensor"));
    _forcesMenu->addAction(_readForcesDataAct);
    _forcesMenu->addAction(_plotNextForcesChunkAct);
    _forcesMenu->addAction(_plotNextLidarChunkAct);
}

void MainWindow::createActions()
{
    _readForcesDataAct = new QAction(tr("Read data"), this);
    _readForcesDataAct->setToolTip(tr("Reads distance data from json file"));
    connect(_readForcesDataAct, &QAction::triggered, this, &MainWindow::readForcesDataSlot);

    _plotNextForcesChunkAct = new QAction(tr("Plot next Forces chunk"), this);
    connect(_plotNextForcesChunkAct, &QAction::triggered, this, &MainWindow::plotNextForcesChunkSlot);

    _plotNextLidarChunkAct = new QAction(tr("Plot next Lidar chunk"), this);
    connect(_plotNextLidarChunkAct, &QAction::triggered, this, &MainWindow::plotNextLidarChunkSlot);
}

void MainWindow::plotForcesData(int chunk_idx)
{
    auto [repulsive, attractive, total] = _forces[chunk_idx];

    chart->removeAllSeries();
    QScatterSeries* repulsiveSeries = new QScatterSeries(chart);//QLineSeries(chart);
    repulsiveSeries->setName("Repulsive force");
    repulsiveSeries->setMarkerSize(15);
    repulsiveSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    for (auto& item : repulsive)
    {
        repulsiveSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }

    chart->addSeries(repulsiveSeries);

    QLineSeries* attractiveSeries = new QLineSeries(chart);
    attractiveSeries->setName("Attractive force");
    //attractiveSeries->setMarkerSize(10);

    QPen pen;
    //pen.setColor(Qt::darkGreen);  // Line color
    pen.setStyle(Qt::DashLine);  // Set dashed line style
    pen.setWidth(5);

    // Optionally, you can customize the dash pattern
    // For example, this pattern alternates between 3px solid and 3px gap
    pen.setDashOffset(3);  // Set offset for the dashes
    pen.setDashPattern({5, 5});

    attractiveSeries->setPen(pen);

    //attractiveSeries->setMarkerShape(QScatterSeries::MarkerShapeStar);

    for (auto& item : attractive)
    {
        attractiveSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }


    QScatterSeries* totalSeries = new QScatterSeries(chart);
    totalSeries->setName("Total force");
    totalSeries->setMarkerShape(QScatterSeries::MarkerShapeTriangle);
    totalSeries->setMarkerSize(25);

    for (auto& item : total)
    {
        totalSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }


    auto safe_angle = std::min_element(std::begin(total), std::end(total),
                                               [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
                                               {
                                                return lhs.distance < rhs.distance;
                              })->angle;
    chart->addSeries(totalSeries);
    chart->addSeries(attractiveSeries);

    chart->createDefaultAxes();
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);

    QAbstractAxis* xAxis = chart->axes(Qt::Horizontal).first();
    xAxis->setTitleFont(legendFont);
    xAxis->setLabelsFont(legendFont);
    xAxis->setTitleText("Angle, t(" + QString::number(current_forces_chunk+1) + "), safe angle: " + QString::number(safe_angle));

    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleFont(legendFont);
    yAxis->setTitleText("Magnitude");
    yAxis->setLabelsFont(legendFont);
}

void MainWindow::plotLidarData(int chunk_idx)
{
    auto lidar_sample = _lidar_data[chunk_idx];

    chart->removeAllSeries();
    QLineSeries* lidarSeries = new QLineSeries(chart);
    lidarSeries->setName("Lidar data");
    QLineSeries* threasholdLine = new QLineSeries;
    threasholdLine->setName("Threashold distance");

    //lidarSeries->setMarkerSize(15);
    //lidarSeries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    for (auto& item : lidar_sample)
    {
        threasholdLine->append(item.angle, _thresholdDistance);
        lidarSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }

    chart->addSeries(lidarSeries);
    chart->addSeries(threasholdLine);

    chart->createDefaultAxes();
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);

    QAbstractAxis* xAxis = chart->axes(Qt::Horizontal).first();
    xAxis->setTitleFont(legendFont);
    xAxis->setLabelsFont(legendFont);
    xAxis->setTitleText("Angle, t(" + QString::number(current_forces_chunk+1)+ ")");

    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleFont(legendFont);
    yAxis->setTitleText("Distance");
    yAxis->setLabelsFont(legendFont);
}
