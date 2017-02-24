#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "../tcp_socket.h"
#include "../udp_socket.h"
#include "../key_control.h"
#include <QMouseEvent>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QDateTime>
#include <QTimer>
#include <QPoint>
#include <QTableWidget>
#include <QProgressBar>
#include "../../common/map_image.h"

#define Myhigh 600
#define Mywidth 600
#define  MIDD 720
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QPixmap *pixmap;
    QProgressBar  *ProgressBar;
    float MyPoint[3600];
    float ShowPoint[MIDD][2];

protected:
    void keyPressEvent(QKeyEvent *e);

private slots:
    void ShowLaser();
    void ShowUltrasonic();
    void on_ultraAll_clicked();
    void on_pBtn_start2connect_clicked(bool checked);
    void on_pBtn_key_control_open_clicked(bool checked);
    void on_lEdit_ip_returnPressed();
    void on_lEdit_port_returnPressed();
    void xTimerUpdate();
    void cmdTimerUpdate();

  //  void on_tableView_activated(const QModelIndex &index);

    void on_pBtn_open_map_clicked();
    //void processPendingDatagram();

    void on_pBtn_open_submap_clicked();
    void on_pBtn_key_Init_IMU_clicked();

    void on_horizontalSlider_valueChanged(int value);
    void on_pBtn_binarization_clicked(bool checked);

    void on_Spin_Sample_Count_valueChanged(int arg1);

    void on_Spin_Filter_Count_valueChanged(int arg1);

private :
    void KeyControlMsgRefalsh(const zw::KeyControlMsg & kMsg);
    void MsgControlRefalsh(void);
    void MsgImuRefalsh(void);
    void MsgUltrasonicRefalsh(void);
    void ClearData();

private:
    bool img_binarization;
    int th_binarization;
    Ui::MainWindow *ui;
    zw::KeyControl *m_keyControl;
    zw::TcpSocket *m_tcpSocketClient;
    zw::UdpSocket *m_udpSocketClient;


    cv::Mat map;
    cv::Mat submap;
    zw::MapImage m_mapImage;

};

#endif // MAINWINDOW_H
