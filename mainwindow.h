#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QtCharts>

#include "deserializer.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void readForcesDataSlot();
    void plotNextForcesChunkSlot();
    void plotNextLidarChunkSlot();

private:
    Ui::MainWindow *ui;
    QMenu *_forcesMenu;
    QAction *_readForcesDataAct;
    QAction *_plotNextForcesChunkAct;
    QAction *_plotNextLidarChunkAct;
    QFont legendFont;

    // plotting primitives
    QChart* chart;
    QChartView* chartView;

    std::vector<Forces> _forces;
    std::vector<LidarSample> _lidar_data;

    int current_forces_chunk = -1;
    int current_lidar_chunk = -1;

    static constexpr double _thresholdDistance{1.};
    //building gui functions
    void createMenus();
    void createActions();

    void plotForcesData(int chunk_idx);
    void plotLidarData(int chunk_idx);
};
#endif // MAINWINDOW_H
