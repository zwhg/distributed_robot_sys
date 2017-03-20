#ifndef NAV_H
#define NAV_H


namespace zw
{

struct CarPose
{
    float x;
    float y;
    float h;
};

struct NavPara
{
    CarPose current;
    CarPose desired;
    bool newGoal;
    bool startNav;
    bool emergeStop;
};

class nav
{
public:
    nav();
};

}
#endif // NAV_H
