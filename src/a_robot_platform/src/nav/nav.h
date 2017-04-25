#ifndef NAV_H
#define NAV_H

#include <vector>

namespace zw
{


constexpr int INF=1000000;
constexpr float PassNodeErr=0.1;


float angle_diff(float a, float b);

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

struct Edge
{
    int start;
    int end;
};

struct Graph
{
    int vertexNum;
    int edgeNum;
    std::vector<CarPose> vertex;
    std::vector<std::vector<int> > adjmap;
    std::vector<Edge> se;
    bool changed;
};




class nav
{
public:
    Graph g;
    bool firstIn;
    float EndNodeDisErr;
    float EndNodeAngErr;
public:
    nav();
    ~nav();
    bool add_vertex(const CarPose & p);
    bool delete_vertex(const int &index);
    bool add_edge(const int& start, const int& end);
    bool delete_edge(const int& start, const int& end);

    bool dijkstra(const int& start, const int& end,std::vector<int>& path);
private:
    void calAdjMap();
};

}
#endif // NAV_H
