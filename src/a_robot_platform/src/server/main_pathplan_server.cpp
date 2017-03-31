#include "main_pathplan_server.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void mapCallback(const nav_msgs::OccupancyGridConstPtr& grid)  //get the map point
{
     //get the height and width of the map
     Height = grid->info.height;
     Width = grid->info.width;
     int8_t   n = grid->info.width*grid->info.height;
     map = new int8_t *[n];
     for(int8_t i=0;i<grid->info.height;i++)
     {
         for(int8_t j=0;j<grid->info.width;j++)
         {
          map[i][j] = grid->data[grid->info.height*i+j];     //convert a one-dimensional array to a two-dimensional array
         }
     }
}
void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped& start_pose)//get the robot pose
{
    int8_t i=0;
    ROS_INFO("the x_pose of robot:%lf",start_pose.pose.pose.position.x);
    ROS_INFO("the y_pose of robot:%lf",start_pose.pose.pose.position.y);
    r_pose_x = start_pose.pose.pose.position.x;
    r_pose_y = start_pose.pose.pose.position.y;
    if(0==i)  //remember the start point of the robot
    {
        r_start_x = r_pose_x;
        r_start_y = r_pose_y;
        i=1;
    }
}
void  Linear_Angular_msg(struct Routes *routes)  //根据路径计算线速度与角速度
{
  //  for(int i = 0;i<strlen(temp);i++)
 //   {}
}
unsigned char within(int x, int y) //判断输入点的坐标是否在地图中
{
    return (x >= 0 && y >= 0 && x < Height && y < Width);
}
// 优先队列基本操作
void initOpen(Open *q)    //优先队列初始化
{
    q->length = 0;                // 队内元素数初始为0
}
void push(Open *q, Close **cls, int x, int y, float g)
{
//向优先队列（Open表）中添加元素
    Close *t;
    int i, mintag;
    cls[x][y].G = g;    //所添加节点的坐标
    cls[x][y].F = cls[x][y].G + cls[x][y].H;
    q->Array[q->length++] = &(cls[x][y]);
    mintag = q->length - 1;
    for (i = 0; i < q->length - 1; i++)
    {
        if (q->Array[i]->F < q->Array[mintag]->F)
        {
            mintag = i;
        }
    }
    t = q->Array[q->length - 1];
    q->Array[q->length - 1] = q->Array[mintag];
    q->Array[mintag] = t;    //将评价函数值最小节点置于队头
}
Close* shift(Open *q)
{
    return q->Array[--q->length];
}
// 地图初始化操作
void initClose(Close **cls, int sx, int sy, int dx, int dy)  //封闭列表，存放路径节点
{
  // 地图Close表初始化配置
    int i, j;
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            cls[i][j].cur = &graph[i][j];                                   // Close表所指节点 ，graph数组存储了节点的相关信息
            cls[i][j].vis = !graph[i][j].reachable;                   // 是否被访问
            cls[i][j].from = NULL;                                         // 所来节点，即他的父节点（从父节点走过来）
            cls[i][j].G = cls[i][j].F = 0;                                    // 初始化G值（起点移动到该点的移动代价）、F值=G+H
            cls[i][j].H = abs(dx - i) + abs(dy - j);                   // 评价函数值H:从指定方格移动到终点B的估算成本
        }
    }
    cls[sx][sy].F = cls[sx][sy].H;                         //起始点评价初始值
    //cls[sy][sy].G = 0;                                       //移步花费代价值
    cls[dx][dy].G = Infinity;
}
void initGraph(int8_t **map, int sx, int sy, int dx, int dy)  //构造地图（程序运行第一步初始化地图）（map[][],0,0,0,0）
{
   //地图发生变化时重新构造地
    int i, j;
    srcX = sx;    //起点X坐标
    srcY = sy;    //起点Y坐标
    dstX = dx;    //终点X坐标
    dstY = dy;    //终点Y坐标
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            graph[i][j].x = i;                      //地图坐标X
            graph[i][j].y = j;                      //地图坐标Y
            graph[i][j].value = map[i][j]; //地图初始化为数组定义值
            graph[i][j].reachable = (graph[i][j].value == Reachable); //节点可到达性,初始化为0的表示可达
            graph[i][j].sur = 0; //邻接节点个数
            if (!graph[i][j].reachable)
            {
                continue;
            }
            if (j > 0)//地图可以向右遍历
            {
                if (graph[i][j - 1].reachable)   // 本节点left节点可以到达
                {
                    graph[i][j].sur |= West;     //本节点西边可达        graph[i][j].sur = graph[i][j]|West(1<<4)  graph[i][j]=00001000
                    graph[i][j - 1].sur |= East; //对应节点东边可达      graph[i][j].sur = graph[i][j]|East(1<<0)  graph[i][j]=00000001
                }
                if (i > 0)//地图可以向上遍历
                {
                    if (graph[i - 1][j - 1].reachable && graph[i - 1][j].reachable &&graph[i][j - 1].reachable)//左上、上、左可以到达==>左上角（西北方向）可以达到
                    {
                        graph[i][j].sur |= North_West;  //西北可达
                        graph[i - 1][j - 1].sur |= South_East; //东南可达
                    }
                }
            }
            if (i > 0)//地图可以向上遍历
            {
                if (graph[i - 1][j].reachable)    // 本节点up节点可以到达
                {
                    graph[i][j].sur |= North;    //本节点北部可达
                    graph[i - 1][j].sur |= South;//对应节点南部可达
                }
                if (j < Width - 1) //地图向上没有达到顶点
                {
                    if (graph[i - 1][j + 1].reachable && graph[i - 1][j].reachable && map[i][j + 1] == Reachable) // 下右、下、右可达==>下右可达
                    {
                        graph[i][j].sur |= North_East; //东北
                        graph[i - 1][j + 1].sur |= South_West; //西南
                    }
                }
            }
        }
    }
}
int bfs()
{
    int times = 0;
    int i, curX, curY, surX, surY;
    unsigned char f = 0, r = 1;
    Close *p;
    Close* q[MaxLength] = { &closelist[srcX][srcY] };

    initClose(closelist, srcX, srcY, dstX, dstY);
    closelist[srcX][srcY].vis = 1;

    while (r != f)
    {
        p = q[f];
        f = (f + 1) % MaxLength;
        curX = p->cur->x;
        curY = p->cur->y;
        for (i = 0; i < 8; i++)
        {
            if (! (p->cur->sur & (1 << i)))
            {
                continue;
            }
            surX = curX + dir[i].x;
            surY = curY + dir[i].y;
            if (! closelist[surX][surY].vis)
            {
                closelist[surX][surY].from = p;
                closelist[surX][surY].vis = 1;
                closelist[surX][surY].G = p->G + 1;
                q[r] = &closelist[surX][surY];
                r = (r + 1) % MaxLength;
            }
        }
        times++;
    }
    return times;
}

int astar()   // A*算法遍历
{
    //int times = 0;
    int i, curX, curY, surX, surY;
    float surG;
    Open q; //Open表
    Close *p; //Close表

    initOpen(&q);
    initClose(closelist, srcX, srcY, dstX, dstY);
    closelist[srcX][srcY].vis = 1;
    push(&q, closelist, srcX, srcY, 0);

    while (q.length)
    {
        p = shift(&q);
        curX = p->cur->x;
        curY = p->cur->y;
        if (!p->H)
        {
            return Sequential;
        }
        for (i = 0; i < 8; i++)
        {
            if (!(p->cur->sur & (1 << i)))
            {
                continue;
            }
            surX = curX + dir[i].x;
            surY = curY + dir[i].y;
            if (!closelist[surX][surY].vis)
            {
                closelist[surX][surY].vis = 1;
                closelist[surX][surY].from = p;
                surG = p->G + sqrt((float)((curX - surX) * (curX - surX) + (curY - surY) * (curY - surY)));
                push(&q, closelist, surX, surY, surG);
            }
        }
    }
    return NoSolution; //无结果
}

void printMap()
{
    int i, j;
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            ROS_INFO("%s", Symbol[graph[i][j].value]);
        }
        ROS_INFO(" ");
    }
    ROS_INFO(" ");
}
Close* getShortest()
{    // 获取最短路径
    int result = astar();
    Close *p, *t, *q = NULL;
    switch(result)
    {
    case Sequential:    //顺序最近
        p = &(closelist[dstX][dstY]);
        while (p)    //转置路径
        {
            t = p->from;
            p->from = q;
            q = p;
            p = t;
        }
        closelist[srcX][srcY].from = q->from;
        return &(closelist[srcX][srcY]);
    case NoSolution:
        return NULL;
    }
    return NULL;
}
int printShortest()  //打印最短路径
{
    Close *p;       //定义Close表指针
    int step = 0;   //步数

    p = getShortest(); //获得最短路径
    start = p;
    if (!p)
    {
        return 0;
    }
    else
    {
        while (p->from)
        {
            graph[p->cur->x][p->cur->y].value = Pass;
            ROS_INFO("(%d,%d)->\n", p->cur->x, p->cur->y);
            //save  the  route  point
            Route[step].x = p->cur->x;
            Route[step].y = p->cur->y;
            p = p->from;
            step++;
        }
        Route[step].x = p->cur->x;
        Route[step].y = p->cur->y;
        ROS_INFO("(%d,%d)\n", p->cur->x, p->cur->y);
        graph[srcX][srcY].value = Source;
        graph[dstX][dstY].value = Destination;
        return step;
    }
}
void clearMap()
{    // Clear Map Marks of Steps
    Close *p = start;
    while (p)
    {
        graph[p->cur->x][p->cur->y].value = Reachable;
        p = p->from;
    }
    graph[srcX][srcY].value = map[srcX][srcY];
    graph[dstX][dstY].value = map[dstX][dstY];
}
void printDepth()
{
    int i, j;
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            if (map[i][j])
            {
                ROS_INFO("%s ", Symbol[graph[i][j].value]);
            }
            else
            {
                ROS_INFO("%2.0lf ", closelist[i][j].G);
            }
        }
        ROS_INFO(" ");
    }
    ROS_INFO(" ");
}
void printSur()
{
    int i, j;
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            ROS_INFO("%02x ", graph[i][j].sur);
        }
        ROS_INFO(" ");
    }
    ROS_INFO(" ");
}
void printH()
{
    int i, j;
    for (i = 0; i < Height; i++)
    {
        for (j = 0; j < Width; j++)
        {
            ROS_INFO("%02d ", closelist[i][j].H);
        }
        ROS_INFO(" ");
    }
    ROS_INFO(" ");
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"main_pathplan_server");
  ROS_INFO("package_name: a_robot_platform  node_name: main_pathplan_server");
  ros::NodeHandle n_map;
  ros::Subscriber map_sub = n_map.subscribe("map",1,mapCallback);                //get the map
  ros::NodeHandle n_pose;
  ros::Subscriber pose_sub = n_pose.subscribe("amcl_pose",2,PoseCallback);    //get the robot pose
  initGraph(map,0,0,0,0);
  //printMap();
  srcX = r_start_x;
  srcY = r_start_y;
  dstX = 10;
  dstY = 10;
  if(srcX>=0&&srcY>=0&&dstX>=0&&dstY>=0)
      {
          ROS_INFO("come in ");
          if (within(srcX, srcY) && within(dstX, dstY)) //如果起点和终点都在地图当中
          {
              if (shortestep = printShortest())
              {
                  ROS_INFO("the minminum step num from(%d,%d) to (%d,%d) is %d\n",
                           srcX, srcY, dstX, dstY, shortestep);
               //   printMap();
                  clearMap();
                  bfs();
                  ROS_INFO((shortestep == closelist[dstX][dstY].G) ? "true" : "false");
                  clearMap();
              }
              else
              {
                  ROS_INFO("from (%d，%d) can not connect  to (%d，%d)\n",
                           srcX, srcY, dstX, dstY);
              }
          }
          else
          {
              ROS_INFO("ERROR");
          }
      }
  ros::spin();
  return 0;
}
