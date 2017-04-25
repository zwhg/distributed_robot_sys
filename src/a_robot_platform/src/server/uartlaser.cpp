#include "uartlaser.h"
#include <ros/ros.h>


namespace zw {

UartLaser ::UartLaser()
{
  pcurr = InitList();
}

UartLaser ::~UartLaser()
{
  DeletList();
}

int32_t UartLaser ::StartScan(void)
{
  static uint8_t scanflags = 0;
  uint8_t startCmd[7] = {0xa5, 0x2C, 0xe1, 0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t dataScanCmd[7] = {0xa5, 0x20, 0xe1, 0xAA, 0xBB, 0xCC, 0xDD};
  int32_t wRet=-1;
  if(pcurr ==NULL)
  {
    std::cout<<"[%s - %d] Initial list failed ! \n"<< __FILE__<<__LINE__;
    return wRet;
  }
  std::cout<<"scanflags="<<scanflags+1<<std::endl;

  if (scanflags == 0)
  {
    wRet = write(fd, startCmd, 7);          //发送开始扫描指令
//    usleep(3000);
//    wRet = write(fd, startCmd, 7);          //发送开始扫描指令
    std::cout<<"wRet="<<wRet<<std::endl;
  }

  usleep(3000);
  wRet = write(fd, dataScanCmd, 7);		//持续请求扫描数据
//  usleep(3000);
//  wRet = write(fd, dataScanCmd, 7);		//持续请求扫描数据
  std::cout<<"wRet="<<wRet<<std::endl;

  createPthread = 1;
  if (scanflags == 0)
  {
    scanflags = 1;
        pthread_create(&id, NULL, UartCreatePthread, (void *)this);
  }
  return wRet;
}

int32_t UartLaser ::GetScanData( double *ang, double *dis, int32_t len, double *spd)
{
  int32_t min = 0;
  int32_t i;

  pthread_mutex_lock(&tMutex);
  pthread_cond_wait(&tConVar, &tMutex);
  min = len > PACKLEN ? PACKLEN : len;
  for (i = 0; i < min; i++)
  {
    ang[i] = angle[i];
    dis[i] = distance[i];
  //  qDebug()<<distance[i];
  }
    *spd  = speed;
    pthread_mutex_unlock(&tMutex);
    Paras::set_distance(dis);
  return min;
}

int32_t UartLaser ::StopScan(void)
{
  int32_t ret=0;
  uint8_t buf[7] = {0xa5, 0x21, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd};  //停止扫描命令
  ret = write(fd, buf, 7);
  usleep(3000);
 // ret = write(fd, buf, 7);
  return  ret;
}

void UartLaser ::Analysis(uint8_t *buf, int nRet)
{
  uint8_t tempbuffer[2048];
  int32_t i,j;
  int32_t clen = 0;

  if (nRet > 0)
  {
    if (!pcurr->start && !pcurr->flag)
    {
      for (i = 0; i < nRet-6; i++)
      {
        if (buf[i] == 0xa5 && buf[i+6] == 0x81)
          break;
      }
      if (i >= nRet-6)
      {
        memcpy(pcurr->data, buf+nRet-6, 6);
        pcurr->flag = 1;
        pcurr->curr = 6;
      }else{
        memcpy(pcurr->data, buf+i, nRet-i);
        pcurr->start = 1;
        pcurr->flag  = 1;
        pcurr->curr  = nRet-i;
      }
    }
    else if (!pcurr->start && pcurr->flag)
    {
      memset(tempbuffer, 0, sizeof(tempbuffer));
      memcpy(tempbuffer, pcurr->data, pcurr->curr);
      memcpy(tempbuffer+pcurr->curr, buf, nRet);
      clen = pcurr->curr+nRet;
      pcurr->start = 0;
      pcurr->end   = 0;
      pcurr->flag  = 0;
      pcurr->curr = 0;
      memset(pcurr->data, 0, PACKSIZE);
      for (i = 0; i < clen-6; i++)
      {
        if (tempbuffer[i] == 0xa5 && tempbuffer[i+6] == 0x81)
          break;
      }
      if (i >= clen-6)
      {
        memcpy(pcurr->data, tempbuffer+clen-6, 6);
        pcurr->flag = 1;
        pcurr->curr = 6;
      }else{
        if (clen-i < PACKSIZE){
          memcpy(pcurr->data, tempbuffer+i, clen-i);
          pcurr->start = 1;
          pcurr->flag  = 1;
          pcurr->curr  = clen-i;
        }else if (clen-i == PACKSIZE){
          memcpy(pcurr->data, tempbuffer+i, clen-i);
          pcurr->start = 1;
          pcurr->flag  = 1;
          pcurr->end   = 1;
          pcurr->curr  += clen-i;
        }else{
          if (tempbuffer[i+PACKSIZE] == 0xa5){
            memcpy(pcurr->data, tempbuffer+i, PACKSIZE);
            pcurr->start = 1;
            pcurr->flag  = 1;
            pcurr->end   = 1;
            pcurr->curr  = PACKSIZE;
            pcurr = pcurr->next;
            pcurr->start = 0;
            pcurr->flag  = 0;
            pcurr->end   = 0;
            pcurr->curr  = 0;
            memset(pcurr->data, 0, PACKSIZE);
            memcpy(pcurr->data, tempbuffer+i+PACKSIZE, clen-i-PACKSIZE);
            pcurr->start = 0;
            pcurr->flag  = 1;
            pcurr->end   = 0;
            pcurr->curr  = clen-i-PACKSIZE;
            pcurr = pcurr->next;
          }else{
            memcpy(pcurr->data, tempbuffer+i+PACKSIZE, clen-i-PACKSIZE);
            pcurr->start = 0;
            pcurr->flag = 1;
            pcurr->curr = clen-i-PACKSIZE;
          }
        }
      }
    }
    else if (pcurr->start && !pcurr->end)
    {
      for (i = 0; i < nRet-6; i++)
      {
        if (buf[i] == 0xa5 && buf[i+6] == 0x81)
          break;
      }
      if (i >= nRet-6)
      {
        if (pcurr->curr+i < PACKSIZE)
        {
          if (pcurr->curr+nRet < PACKSIZE)
          {
            memcpy(pcurr->data+pcurr->curr, buf, nRet);
            pcurr->curr += nRet;
          }
          else if (pcurr->curr+nRet == PACKSIZE)
          {
            memcpy(pcurr->data+pcurr->curr, buf, nRet);
            pcurr->curr += nRet;
            pcurr->end   = 1;
          }
          else
          {
            clen = PACKSIZE-pcurr->curr;
            if (buf[clen] == 0xa5)
            {
              memcpy(pcurr->data+pcurr->curr, buf, clen);
              pcurr->end   = 1;
              pcurr->curr += clen;
              pcurr = pcurr->next;
              pcurr->start = 0;
              pcurr->end   = 0;
              pcurr->flag  = 0;
              memset(pcurr->data, 0, PACKSIZE);
              memcpy(pcurr->data, buf+clen, nRet-clen);
              pcurr->start = 0;
              pcurr->curr  = nRet-clen;
              pcurr->end   = 0;
              pcurr->flag  = 1;
              pcurr = pcurr->next;
            }
            else
            {
              pcurr->start = 0;
              pcurr->end   = 0;
              pcurr->flag  = 0;
              memset(pcurr->data, 0, PACKSIZE);
              memcpy(pcurr->data, buf+nRet-3, 3);
              pcurr->start = 0;
              pcurr->flag = 1;
              pcurr->curr = 3;
            }
          }
        }//
        else if (pcurr->curr+i == PACKSIZE)
        {
          if (buf[i] == 0xa5)
          {
            memcpy(pcurr->data+pcurr->curr, buf, i);
            pcurr->curr += i;
            pcurr->end = 1;
            pcurr = pcurr->next;
            pcurr->start = 0;
            pcurr->end   = 0;
            pcurr->flag  = 0;
            memset(pcurr->data, 0, PACKSIZE);
            memcpy(pcurr->data, buf+i, nRet-i);
            pcurr->start = 0;/* no start*/
            pcurr->flag = 1;
            pcurr->curr = nRet-i;
            pcurr = pcurr->next;
          }
          else
          {
            pcurr->start = 0;
            pcurr->end   = 0;
            pcurr->flag  = 0;
            memset(pcurr->data, 0, PACKSIZE);
            memcpy(pcurr->data, buf+nRet-3, 3);
            pcurr->start = 0;
            pcurr->flag = 1;
            pcurr->curr = 3;
          }
        }
        else
        {
          pcurr->start = 0;
          pcurr->end   = 0;
          pcurr->flag  = 0;
          memset(pcurr->data, 0, PACKSIZE);
          memcpy(pcurr->data, buf+nRet-6, 6);
          pcurr->start = 0;
          pcurr->flag = 1;
          pcurr->curr = 6;
        }
      }
      else
      {
        if (pcurr->curr+i != PACKSIZE)
        {
          pcurr->start = 0;
          pcurr->end   = 0;
          pcurr->flag  = 0;
          memset(pcurr->data, 0, PACKSIZE);

          memcpy(pcurr->data, buf+i, nRet-i);
          pcurr->start = 1;
          pcurr->flag = 1;
          pcurr->curr = nRet-i;
        }
        else
        {
          memcpy(pcurr->data+pcurr->curr, buf, i);
          pcurr->start = 1;
          pcurr->flag = 1;
          pcurr->end  = 1;
          pcurr->curr += i;
          pcurr = pcurr->next;
          memcpy(pcurr->data, buf+i, nRet-i);
          pcurr->start = 1;
          pcurr->flag = 1;
          pcurr->end  = 0;
          pcurr->curr = nRet-i;
          pcurr = pcurr->next;
        }
      }
    }
    if (pcurr->start && pcurr->end)
    {
      pthread_mutex_lock(&tMutex);
      pthread_cond_signal(&tConVar);
      LsParameter(pcurr->data, angle, distance, pcurr->curr);
      pcurr->start = 0;
      pcurr->end   = 0;
      pcurr->flag  = 0;
      memset(pcurr->data, 0, PACKSIZE);
      pcurr = pcurr->next;
      pthread_mutex_unlock(&tMutex); 
    }
  }
}

//将封装好的一帧数据解包
int32_t UartLaser ::LsParameter(uint8_t *data, double *angle, double *dist, int32_t len)
{
  rplidar_response_measurement_node_t *curr;
  uint8_t *tmp ;
  uint32_t i,j;
  if (  (data[0] == 0xA5) && (data[6] == 0x81) &&
    (data[len-4] == 0xaa) &&(data[len-1] == 0xdd)  )
  {
    tmp =data+7;
    speed =data[1]/15;
    curr = (rplidar_response_measurement_node_t *)tmp;
    for (i = 7, j = 0; i < len-4;  curr++, i += 5, j++)
    {
      angle[j] = curr->angle_q6_checkbit/10.0;
      dist[j] = curr->distance_q2/1.0;
    }
    return j;
  }
  return 0;
}


void *UartLaser ::DoPthread(void)
{
  uint8_t buf[READMAX];
  static int8_t cnt=0;
  while(createPthread)
  {
    cnt ++ ;
    bzero(buf,READMAX);
    if(cnt==33){
  //      std::cout<<"laser ok\n";
        cnt=0;
    }
    int32_t nRet = read(fd, buf, READMAX);  
    if( nRet >0 )
    {
      Analysis(buf, nRet);
    }
    usleep(30000);
  }
}

/**
初始化一个链表节点
*/
basedata_t *UartLaser ::CreateList(void)
{
  basedata_t *head;

  head = (basedata_t *)malloc(sizeof(basedata_t));
  if (NULL == head)
    return NULL;
  head->flag = 0;
  head->start = 0;
  head->end  = 0;
  head->curr = 0;
  head->next = NULL;

  return head;
}

/**
初始化一个两节点的循环链表
*/
basedata_t *UartLaser ::InitList(void)
{
  basedata_t *head,*p;

  head = CreateList();
  if (NULL == head)
    return NULL;
  p = CreateList();
  if (NULL == p){
    free(head);
    return NULL;
  }
  head->next = p;
  p->next = head;

  return head;
}

void UartLaser ::DeletList(void)
{
  if(NULL != pcurr->next)
    free(pcurr->next);
  if(NULL !=pcurr)
    free(pcurr);
}

}

