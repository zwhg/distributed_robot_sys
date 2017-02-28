#include "map_process.h"
#include <ros/ros.h>

namespace zw {


void map_filter(char *out,uint32_t w, uint32_t h)
{
  int index;
  for(int x=1;x<w-1;x++)
  {
       index= GetGridIndexOfMap(w,x,0);
       if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
          (out[index+1]!=kFreeGrid)&&(out[index+w]!=kFreeGrid))
           out[index] =kUnknownGrid;
       index= GetGridIndexOfMap(w,x,h-1);
       if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
          (out[index+1]!=kFreeGrid)&&(out[index-w]!=kFreeGrid))
           out[index] =kUnknownGrid;
  }
  for(int y=1;y<h-1;y++)
  {
       index= GetGridIndexOfMap(w,0,y);
       if((out[index]==kFreeGrid)&&(out[index-w]!=kFreeGrid)&&
          (out[index+w]!=kFreeGrid)&&(out[index+1]!=kFreeGrid))
           out[index] =kUnknownGrid;
       index= GetGridIndexOfMap(w,w-1,y);
       if((out[index]==kFreeGrid)&&(out[index-w]!=kFreeGrid)&&
          (out[index+w]!=kFreeGrid)&&(out[index-1]!=kFreeGrid))
           out[index] =kUnknownGrid;
  }
  for(int y=1;y<h-1;y++)
  {
      for(int x=1;x<w-1;x++)
      {
          index= GetGridIndexOfMap(w,x,y);
          if((out[index]==kFreeGrid)&&(out[index-1]!=kFreeGrid)&&
             (out[index+1]!=kFreeGrid)&&(out[index+w]!=kFreeGrid)&&
             (out[index-w]!=kFreeGrid))
              out[index] =kUnknownGrid;
      }
  }
}

bool CellInfo_cmpdis_err(const CellInfo& c1,const CellInfo & c2)
 {
    return c1.dis_err <c2.dis_err;
 }

bool CellInfo_cmpdis_grade(const CellInfo& c1,const CellInfo & c2)
{
      return c1.grade < c2.grade;
}

MapProcess::MapProcess()
{
    valid_cell_count[0]=0;
    valid_cell_count[1]=0;
    valid_cell_count[2]=0;
    valid_cell_count[3]=0;
    valid_cell_count[4]=0;
    free_grid_Cell.resize(0);
    map_resolution =0.05;
}

MapProcess::~MapProcess()
{

}

void MapProcess::GetBinaryAndSample(const nav_msgs::OccupancyGridConstPtr& grid ,int th_occ ,int th_free ,int num )
{
   free_grid_Cell.resize(0);
   map_resolution =0.05;
   valid_cell_count[0]=grid->info.width*grid->info.height ;
   valid_cell_count[1]=0;
   valid_cell_count[2]=0;
   valid_cell_count[3]=0;
   valid_cell_count[4]=0;

   int w=grid->info.width/num;
   int h=grid->info.height/num;
   map_resolution = grid->info.resolution *num;
   char *mapdata = new char[w*h];
   int index=0;
   for(int y=0;y<h;y++)
   {
       for(int x=0; x<w;x++)
       {
          int sum=0;
          for(int j=0;j<num;j++)
          {
              for(int i=0;i<num;i++)
              {
                  index= GetGridIndexOfMap(grid->info.width,(x*num+i),(y*num+j));
                  sum+= grid->data[index];
              }
          }
          if(sum>=th_occ)
              mapdata[y*w+x] = kOccGrid;
          else if(sum <th_free)
              mapdata[y*w+x] = kUnknownGrid;
          else
              mapdata[y*w+x] =kFreeGrid;
       }
   }

   map_filter(mapdata,w,h);
   valid_cell_count[1]= GetFreeSpcaceIndices(mapdata,w,h);
   CalNeighbour(mapdata,w,h ,map_resolution);

   filter_map.info.resolution = map_resolution;
   filter_map.info.width = w;
   filter_map.info.height = h;
   filter_map.info.map_load_time=ros::Time::now();
   filter_map.header.frame_id="map";
   filter_map.header.stamp=ros::Time::now();
   filter_map.data.resize(w*h,-1);

   filter_map.info.origin.position.x =grid->info.origin.position.x;
   filter_map.info.origin.position.y =grid->info.origin.position.y;
   filter_map.info.origin.position.z =0;

   for (size_t k = 0; k < w*h; ++k)
       filter_map.data[k]=mapdata[k];

   delete mapdata;

   ROS_INFO("map info %d X %d map @ %3.2lf m/cell",filter_map.info.width,
            filter_map.info.height,filter_map.info.resolution);
}

int MapProcess::GetFreeSpcaceIndices(const char *grid,int w,int h)
{
    CellInfo cell={0,0,0,0,10000,0,0,{0,0,0,0,0,0,0,0}};

    for(int j = 0; j <h; j++)
        for(int i = 0; i < w; i++)
        {
            if(grid[GetGridIndexOfMap(w,i,j)]==kFreeGrid)
            {
                cell.x=i;
                cell.y=j;
                free_grid_Cell.push_back(cell);
            }
        }
    return (int)free_grid_Cell.size();
}

void MapProcess::CalNeighbour(const char *grid,int w,int h ,float resolution)
{
  //  int min_count = floor(kMinLaserRange/resolution);
    float res;
    int max_count;
    for(int i=0;i<valid_cell_count[1];i++)
    {
        int k=1;
        int x= free_grid_Cell[i].x;
        int y= free_grid_Cell[i].y;
        float sum=0;
        int count=0;
        res=resolution;
        max_count = floor(kMaxLaserRange/res);
        while(k<=max_count)
        {
            if((y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[0] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[2] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
               break;

        }
        k=1;
        while(k<=max_count)
        {
            if((y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[4] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;

        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[6] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
               break;
        }

        res=resolution*1.414213;
        max_count = floor(kMaxLaserRange/res);
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w &&(y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[1] = k*res ;
                     sum += k*res;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break ;

        }
        k=1;
        while(k<=max_count)
        {
            if((x+k)<w &&(y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x+k,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[3] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0 &&(y-k)>=0)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y-k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[5] = k*res ;
                     sum += k*res ;
                     count++;
                     break;
                 }
                 k++;
            }
            else
                break;
        }
        k=1;
        while(k<=max_count)
        {
            if((x-k)>=0 &&(y+k)<h)
            {
                 if(grid[GetGridIndexOfMap(w,x-k,y+k)]==kOccGrid)
                 {
                     free_grid_Cell[i].neighbour[7] = k*res ;
                     sum += k*res;
                     count++;
                     break;
                 }
                 k++;
            }
            else
             break;
        }

        if(count!=0)
            free_grid_Cell[i].dis_avg =sum/count;
        else
            free_grid_Cell[i].dis_avg =10000.0;

    //    ROS_INFO("[%4d %4d %4.2f]",x,y,free_grid_Cell[i].dis_avg);
    }

//    for(int i=0;i<5;i++)
//    ROS_INFO("[x,y]=[%d,%d]  [%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f]",
//        free_grid_Cell[i].x,free_grid_Cell[i].y,
//        free_grid_Cell[i].neighbour[0],free_grid_Cell[i].neighbour[1],
//        free_grid_Cell[i].neighbour[2],free_grid_Cell[i].neighbour[3],
//        free_grid_Cell[i].neighbour[4],free_grid_Cell[i].neighbour[5],
//        free_grid_Cell[i].neighbour[6],free_grid_Cell[i].neighbour[7]);
}

void MapProcess::CalScan(const sensor_msgs::LaserScanConstPtr& scan ,float perr)
{
    size_t size = scan->ranges.size();
    float angle = scan->angle_min;

    float sum=0;
    int count=0;

    for (size_t i = 0; i < size; ++i)
    {
        float dist = scan->ranges[i];
        if ( (dist >kMinLaserRange) && (dist < kMaxLaserRange))
        {
            sum +=dist ;
            count ++;
        }
        angle += scan->angle_increment;
    }
    singleScan.dis_avg =sum/count ;

    int pcnt =valid_cell_count[1];
    for(int i=0;i<pcnt;i++)
    {
      free_grid_Cell[i].dis_err =fabs(free_grid_Cell[i].dis_avg- singleScan.dis_avg);
    }

    std::sort(free_grid_Cell.begin(),free_grid_Cell.end(),CellInfo_cmpdis_err);
//    for(int i=0;i<free_space_count;i++)
//      ROS_INFO("%f",free_grid_Cell[i].dis_err);

    pcnt *= perr;
    for(int i=0;i< pcnt;i++)
      free_grid_Cell[i].status =1;

    valid_cell_count[2]=pcnt;

//    ROS_INFO("l_dis=%6.2f min_er=%6.2f ma_er=%6.2f",singleScan.dis_avg,
//             free_grid_Cell[0].dis_err,free_grid_Cell[pcnt-1].dis_err);

    calHeading(scan,10,0.5);

    ROS_INFO("optimize-3 cell= %d/%d/%d", valid_cell_count[3],
              valid_cell_count[2],valid_cell_count[1]);
}

geometry_msgs::Point32 MapProcess::GetPoint(const CellInfo & cell ,const nav_msgs::OccupancyGrid& map)
{
    geometry_msgs:: Point32 p;

    p.x= map.info.origin.position.x +(cell.x+0.5)*map.info.resolution ;
    p.y= map.info.origin.position.y +(cell.y+0.5)*map.info.resolution ;
    p.z=0;
    return p;
}

void  MapProcess::calHeading(const sensor_msgs::LaserScanConstPtr& scan,int skip,float perr)
{
    float neighbour[8]={0};
    int size = (int)scan->ranges.size();
    float ang= scan->angle_min;
    int pcnt= valid_cell_count[2];
    int cnt= size/skip;

    for(int i=0;i<cnt;i++)
    {
        for(int j=0;j<8;j++)
        {
          int index = i*skip+j*size/9 ;
          if(index > size)
            index =index/((index/size)*size);
          float dist = scan->ranges[index];
          if ( (dist >kMinLaserRange) && (dist < kMaxLaserRange))
              neighbour[j]=dist;
        }
        for(int k=0;k<pcnt;k++)
        {
            float sum=0;
            float avg=0;
            for(int j=0;j<8;j++)
            {
                avg += neighbour[j];
                sum += free_grid_Cell[k].neighbour[j]*neighbour[j];
            }
            avg /=8;
            avg = fabs(free_grid_Cell[k].dis_avg - avg);
//            if(sum>free_grid_Cell[k].grade)
//                free_grid_Cell[k].grade =sum ;
            if(avg<free_grid_Cell[k].grade)
              free_grid_Cell[k].grade = avg;
        }
    }
    std::sort(free_grid_Cell.begin(),free_grid_Cell.end(), CellInfo_cmpdis_grade);

    pcnt *= perr;
    for(int i=0;i< pcnt;i++)
    {
      free_grid_Cell[i].status =2;
    //  ROS_INFO("%f",free_grid_Cell[i].grade);
    }

    valid_cell_count[3] =pcnt;
}


}

