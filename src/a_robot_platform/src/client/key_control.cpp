#include <QObject>
#include <QTimer>
#include "../tcp_socket.h"
#include "../key_control.h"
#include "../../common/common.h"
#include "../../common/paras.h"


namespace zw {

    KeyControl::KeyControl(QObject *parent):
        QObject(parent)

    {
        keyFlag =0x00;
        keyControl =false;
        kMsg={0,0,0,0};
        QTimer *m_timer =new QTimer();
        QObject::connect(m_timer,SIGNAL(timeout()),this,SLOT(on_TimerUpdate()));
        m_timer->start(50);
    }

    KeyControl:: ~KeyControl()
    {

    }

    void KeyControl::on_TimerUpdate(void)
    {
        Float2Int32 f2is,f2io;
        int32_t dat[2];
        ParaGetSet packInfo={W_REGISTER,2, CONTROL,dat};
        Paras m_para;
        if(keyControl) {
            if (((keyFlag & 0x01) == 0x00) && ((keyFlag & 0x02) == 0x00))  //既没有按前进，也没有按后退
            {
              if(kMsg.e_speed>0){
                  if(kMsg.e_speed>SPEED_STEP)
                      kMsg.e_speed -=SPEED_STEP;
                  else
                      kMsg.e_speed =0;
              }else if(kMsg.e_speed<0){
                  if(kMsg.e_speed<-SPEED_STEP)
                      kMsg.e_speed +=SPEED_STEP;
                  else
                      kMsg.e_speed=0;
              }
            }
            if (((keyFlag & 0x04) == 0x00) && ((keyFlag & 0x08) == 0x00))  //既没有按左转也没有按右转
            {
                if(kMsg.e_omega>0){
                    if(kMsg.e_omega>OMEGA_STEP)
                        kMsg.e_omega -=OMEGA_STEP;
                    else
                        kMsg.e_omega=0;
                }else if(kMsg.e_omega<0){
                    if(kMsg.e_omega<-OMEGA_STEP)
                        kMsg.e_omega +=OMEGA_STEP;
                    else
                       kMsg.e_omega=0;
                }
            }
            f2is.f=kMsg.e_speed;
            f2io.f=kMsg.e_omega;
//            f2is.f=0.5;
//            f2io.f=-0.2;
            dat[0]=f2is.i;
            dat[1]=f2io.i;
            m_para.SetAddressValue(packInfo);
            keyFlag=0x00;

            //qDebug()<< f2is.f<<f2io.f;
        }
//        packInfo={R_REGISTER,2, MSG_CONTROL,dat};
//        m_para.GetAddressValue(packInfo);
//        f2is.i=dat[0];
//        f2io.i=dat[1];
//        kMsg.a_speed=f2is.f;
//        kMsg.a_omega=f2io.f;
    }


    void KeyControl::ControlKeyDown(QKeyEvent *e)
    {
        switch(e->key())
        {
        case KEY_UP: {
            kMsg.e_speed +=SPEED_STEP;
            if(kMsg.e_speed >kMsg.maxSpeed)
                kMsg.e_speed=kMsg.maxSpeed;
            keyFlag |=0x01;
            break;
        }
        case KEY_DOWN: {
            kMsg.e_speed -=SPEED_STEP;
            if(kMsg.e_speed<-kMsg.maxSpeed)
                kMsg.e_speed=-kMsg.maxSpeed;
            keyFlag |=0x02;
            break;
        }
        case KEY_LEFT: {
            kMsg.e_omega +=OMEGA_STEP;
            if(kMsg.e_omega>kMsg.maxOmega)
                kMsg.e_omega=kMsg.maxOmega;
            keyFlag |=0x04;
            break;
        }
        case KEY_RIGHT: {
            kMsg.e_omega -=OMEGA_STEP;
            if(kMsg.e_omega<-kMsg.maxOmega)
                kMsg.e_omega=-kMsg.maxOmega;
            keyFlag |=0x08;
            break;
        }
        case KEY_SPEED_ADD: {
            kMsg.maxSpeed +=SET_MAX_SPEED_STEP;
            if(kMsg.maxSpeed>MAX_SPEED_CMD)
                kMsg.maxSpeed=MAX_SPEED_CMD;
            break;
        }
        case KEY_SPEED_SUB: {
            kMsg.maxSpeed -=SET_MAX_SPEED_STEP;
            if(kMsg.maxSpeed<0)
                kMsg.maxSpeed=0;
            break;
        }
        case KEY_OMEGA_ADD: {
            kMsg.maxOmega +=SET_MAX_OMEGA_STEP;
            if(kMsg.maxOmega>MAX_OMEGA_CMD)
                kMsg.maxOmega =MAX_OMEGA_CMD;
            break;
        }
        case KEY_OMEGA_SUB: {
            kMsg.maxOmega -=SET_MAX_OMEGA_STEP;
            if(kMsg.maxOmega<0)
                kMsg.maxOmega=0;
            break;
        }
        default:break;
        }
    }
}
