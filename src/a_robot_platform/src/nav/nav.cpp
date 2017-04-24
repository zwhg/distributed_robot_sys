#include "nav.h"
#include <math.h>

namespace zw {


static inline  float normalize(float z)
{
  return atan2(sin(z),cos(z));
}

float angle_diff(float a, float b)
{
  float d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

nav::nav()
{
 g.vertexNum=0;
 g.edgeNum=0;
 g.changed = false;
 firstIn=true;
}

nav::~nav()
{
    g.vertex.clear();
    g.se.clear();
    g.adjmap.clear();
}

bool nav::add_vertex(const CarPose & p)
{
    g.vertex.push_back(p);
    g.vertexNum ++;
    g.changed = true ;
    return true;
}

bool nav::delete_vertex(const int &index)
{
    if( (index<0) || (index>=g.vertexNum))
        return  false;
    g.vertexNum --;
    g.vertex.erase(g.vertex.begin()+index);
    for(int i=0;i<g.edgeNum;i++)
    {
        if(index==g.se[i].start || index == g.se[i].end)
        {
            g.edgeNum --;
            g.se.erase(g.se.begin()+i);
            i--;
            continue;
        }
        if(index <  g.se[i].start)
             g.se[i].start --;
        if(index <  g.se[i].end)
             g.se[i].end --;
    }
    g.changed = true ;
    return true;
}

bool nav::add_edge(const int& start, const int& end)
{
    if(start == end || start<0 ||end<0 || start>=g.edgeNum || end>=g.edgeNum)
        return  false;
    Edge e{start,end};
    g.se.push_back(e);
    g.edgeNum ++ ;
    g.changed = true ;
    return true;
}

bool nav::delete_edge(const int& start, const int& end)
{
    if(start == end || start<0 ||end<0 || start>=g.edgeNum || end>=g.edgeNum)
        return false;
    for(int i=0;i<g.edgeNum;i++)
    {
        if(start==g.se[i].start && end ==g.se[i].end)
        {
            g.edgeNum -- ;
            g.se.erase(g.se.begin()+i);
            g.changed = true ;
            break;
        }
    }
    return true;
}

void nav::calAdjMap()
{
    if(g.changed)
    {
        g.changed =false;
        g.adjmap.resize(g.vertexNum,std::vector<int>(g.vertexNum,-1));
        for(int i=0 ;i<g.edgeNum ;i++)
        {
             Edge se{g.se[i].start,g.se[i].end};
             CarPose p1,p2;
             p1 =g.vertex[se.start];
             p2 =g.vertex[se.end];
             int dis =  floor(sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))*100);
             g.adjmap[se.start][se.end]=g.adjmap[se.end][se.start]=dis;
        }
    }
}

//负边被认作不联通
//dist 出发点到各点的最短路径长度
//path 路径上到达该点的前一个点
bool nav::dijkstra(const int& start, const int& end, std::vector<int> & path)
{
    calAdjMap();
    if(start == end || start<0 ||end<0 || start>=g.edgeNum || end>=g.edgeNum)
        return false;

    const int &NODE= g.vertexNum;
    std::vector<int> dist;
    dist.assign(NODE,-1);//初始化距离为未知
    path.assign(NODE,-1);//初始化路径为未知
    std::vector<bool> flag(NODE,false);//标志数组，判断是否处理过
    dist[start]=0;//出发点到自身路径长度为0
    while(1)
    {
        int v=-1;//初始化为未知
        for(int i=0; i!=NODE; ++i)
          if(!flag[i]&&dist[i]>=0)//寻找未被处理过且
              if(v<0||dist[i]<dist[v])//距离最小的点
                  v=i;
        if(v==end)
            return true;

        if(v<0) //所有联通的点都被处理过
            return false;
        flag[v]=1;//标记
        for(int i=0; i!=NODE; ++i)
        {
            if(g.adjmap[v][i]>=0)//有联通路径且
            {
                if(dist[i]<0||dist[v]+g.adjmap[v][i]<dist[i])//不满足三角不等式
                {
                    dist[i]=dist[v]+g.adjmap[v][i];//更新
                    path[i]=v;//记录路径
                }
            }
        }
    }
}


}
