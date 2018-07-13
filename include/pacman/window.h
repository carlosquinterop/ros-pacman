#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QObject>

class QSlider;
class QPushButton;

class GLWidget;
class MainWindow;

class Window : public QWidget
{
    Q_OBJECT

public:
    Window(MainWindow *mw);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void playSlot();
    
private:
    GLWidget *glWidget;
    QPushButton *playBtn;
    MainWindow *mainWindow;
    
signals:
    void arrowKey(int key);
};

#endif
