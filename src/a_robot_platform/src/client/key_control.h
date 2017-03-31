#ifndef KEY_CONTROL_H
#define KEY_CONTROL_H

#include <QKeyEvent>

namespace zw {

#define KEY_UP      (Qt::Key_W)
#define KEY_DOWN    (Qt::Key_S)
#define KEY_LEFT    (Qt::Key_A)
#define KEY_RIGHT   (Qt::Key_D)
#define KEY_SPEED_ADD   (Qt::Key_Z)
#define KEY_SPEED_SUB   (Qt::Key_X)
#define KEY_OMEGA_ADD   (Qt::Key_C)
#define KEY_OMEGA_SUB   (Qt::Key_V)

#define SPEED_STEP  0.02
#define OMEGA_STEP  0.1

#define SET_MAX_SPEED_STEP 0.1
#define SET_MAX_OMEGA_STEP 0.5

#define MAX_SPEED_CMD	0.5     // m/s
#define MAX_OMEGA_CMD   1     // rad/s


#pragma pack(4)
typedef struct
{
    float maxSpeed;
    float e_speed;
 //   float a_speed;
    float maxOmega;
    float e_omega;
 //   float a_omega;
}KeyControlMsg;

class KeyControl :public QObject{

    Q_OBJECT
    private:   
        unsigned char keyFlag;
    public:
        KeyControlMsg kMsg;
        bool keyControl;

    public:
        explicit  KeyControl(QObject *parent=0 );
        ~KeyControl();
        void ControlKeyDown(QKeyEvent *e);

     private slots:
        void on_TimerUpdate(void);
    };

}


#endif // KEY_CONTROL_H
