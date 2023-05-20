#ifndef LocatingPanel_H
#define LocatingPanel_H

#include <QWidget>

namespace Ui {
class LocatingPanel;
}

class LocatingPanel : public QWidget
{
    Q_OBJECT

public:
    explicit LocatingPanel(QWidget *parent = nullptr);
    ~LocatingPanel();
signals:
    void adjustXYSignal();
    void adjustYawSignal();

public:
    int compensate_x_ = 0;
    int compensate_y_ = 0;
    int compensate_yaw_;

private slots:

    void on_button_up_clicked();

    void on_button_down_clicked();

    void on_button_left_clicked();

    void on_button_right_clicked();

    void on_dial_yaw_valueChanged(int value);

    void on_button_add_yaw_clicked();

    void on_button_sub_yaw_clicked();

private:
    Ui::LocatingPanel *ui;
};

#endif // LocatingPanel_H
