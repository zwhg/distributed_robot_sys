#ifndef MAIN_PATHPLAN_SERVER_H
#define MAIN_PATHPLAN_SERVER_H
#include <ros/ros.h>

int8_t  r_pose_x = 0,r_pose_y=0;                   //the pose of robot
int8_t  r_start_x = 0,r_start_y=0;                    //the start of  A*
int8_t  r_end_x = 0,r_end_y = 0;                    //the end of A*

struct Routes
{
    int8_t x;
    int8_t y;
};
struct Routes *Route = new Routes;//save the necessary point on the path
#define MaxLength 100    //用于优先队列（Open表）的数组
//#define Height     15       //地图高度
//#define Width      20       //地图宽度
int8_t Height = 0;
int8_t Width = 0;

#define Reachable   0    //可以到达的结点
#define Bar         1         //障碍物
#define Pass        2        //需要走的步数
#define Source      3        //起点
#define Destination 4     //终点

#define Sequential  0    //顺序遍历
#define NoSolution  2    //无解决方案
#define Infinity    0xfffffff  //无穷大

#define East       (1 << 0)  //1
#define South_East (1 << 1)  //2
#define South      (1 << 2)  //4
#define South_West (1 << 3)  //8
#define West       (1 << 4)  //16
#define North_West (1 << 5)  //32
#define North      (1 << 6)  //64
#define North_East (1 << 7)  //128
/*
    int8_t map[Height][Width] ={
    {0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,1,1},
    {0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1},
    {0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,1},
    {0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,1},
    {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
    {0,1,0,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0}
};  //the points of the grid map
*/
int8_t  **map=NULL;  //save the map data
typedef struct
{
    signed char x, y;
} Point;

const Point dir[8] =
{                //                            --------------------------
    {0, 1},   // East                     [-1 1]     东[0 1]    [1 1]
    {1, 1},   // South_East         [-1 0]北     [0 0]    南[1 0]
    {1, 0},   // South                  [-1 -1]    西[0 -1]   [1 -1]
    {1, -1},  // South_West       --------------------------
    {0, -1},  // West
    {-1, -1}, // North_West
    {-1, 0},  // North
    {-1, 1}   // North_East
};

typedef struct  //地图结构体
{
    int x, y;   //地图坐标
    unsigned char reachable, sur, value; //可达性、邻接节点个数、值(0\1)
} MapNode;

typedef struct Close       //Close表
{
    MapNode *cur;          //指向地图结构体的指针
    char vis;
    struct Close *from;    //记录父节点
    float F, G;
    int H;
} Close;

typedef struct      //优先队列（Open表）
{
    int length;                 //当前队列的长度
    Close* Array[MaxLength];    //评价节点的指针
} Open;
static MapNode **graph;  //存储地图中的点
static int srcX, srcY, dstX, dstY;    //起始点、终点
static Close **closelist;

const char Symbol[5][3] = { "o", "X", "i", "i", "i" };

static Close *start;
static int shortestep;
#endif // MAIN_PATHPLAN_SERVER_H
