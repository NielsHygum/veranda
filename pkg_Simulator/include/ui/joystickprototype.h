#ifndef JOYSTICKPROTOTYPE_H
#define JOYSTICKPROTOTYPE_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QScrollBar>
#include <QDebug>


namespace Ui {
class JoystickWidget;
class JoystickPrototype;
class RotationWidget;
class KeyReciever;
}

class JoystickWidget : public QOpenGLWidget /*Joystickprototype_If*/
{
    Q_OBJECT

public:
    JoystickWidget(QWidget *parent);
    bool m_MousePressed, m_MouseUp, m_MouseDown, m_MouseEast, m_MouseWest;
    void setCenter();
    double xVector, yVector;
    int xCenter, yCenter;

protected:
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void paintEvent(QPaintEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

private:
    int xCorner, yCorner;
    int radius;

signals:
    void joystickMoved();
};

class RotationWidget : public QScrollBar
{
    Q_OBJECT

public:
    RotationWidget(QWidget *parent);
    bool m_MousePressed, m_MouseLeft, m_MouseRight;
    void addValue(int amount);
    void resetValue();
    int value;
    double zVector;

protected:
    void mouseReleaseEvent(QMouseEvent * event);
};

class JoystickPrototype : public QWidget
{
    Q_OBJECT

public:
    JoystickPrototype(QWindow *parent);
    JoystickWidget *joystickWidget;
    RotationWidget *rotationWidget;

    void closeEvent(QCloseEvent *)
    {
        emit joystickClosed();
    }

public slots:
    void joystickMoved();

protected:
    virtual void keyPressEvent(QKeyEvent * event);
    virtual void keyReleaseEvent(QKeyEvent * event);
    int up;
    int east;
    int west;
    int down;
    int left;
    int right;

private:
    Ui::JoystickPrototype *joystickUi;

signals:
    void joystickMoved(double x, double y, double z, QString channel);
    void joystickButtonPress(int key, QString channel);
    void joystickButtonRelease(int key, QString channel);
    void joystickClosed();

};

#endif // JOYSTICKPROTOTYPE_H