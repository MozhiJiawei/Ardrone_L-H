// ImgRecon.cpp: implementation of the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

#include "ImgRecon.h"
#include <iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ImgRecon::ImgRecon(IplImage *img)
{
  SamLoca[0]="samples\\sample0.bmp";SamLoca[5]="samples\\sample5.bmp";//样本存放地址
  SamLoca[1]="samples\\sample1.bmp";SamLoca[6]="samples\\sample6.bmp";
  SamLoca[2]="samples\\sample2.bmp";SamLoca[7]="samples\\sample7.bmp";
  SamLoca[3]="samples\\sample3.bmp";SamLoca[8]="samples\\sample8.bmp";
  SamLoca[4]="samples\\sample4.bmp";SamLoca[9]="samples\\sample9.bmp";

  src = cvCreateImage( cvSize(640,360), IPL_DEPTH_8U, 1);
  if(img != NULL){
    ReInit(img);
  }
}

ImgRecon::~ImgRecon()
{

}

//更改下一张图片
void ImgRecon::ReInit(IplImage *img)
{	
  NumResult = -2;
  Center = cvPoint(320,180);
  ConArea = 0;
  ConExist = 0;


  IplImage 
    //*src,//灰度图，二值化，腐蚀
    *dst,//复制src后找轮廓
    *dst1;//画拟合曲线，找角点
  //src = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  dst1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);

  cvCvtColor( img, src, CV_BGR2GRAY );//转灰img->src
  cvThreshold( src, src, 130, 255, CV_THRESH_BINARY);//二值化src
  cvErode( src, src, NULL, 1);//腐蚀src
  cvCopy( src, dst);//备份src->dst

  //找轮廓dst
  CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
  CvSeq *contour = 0, *cont=0, *contemp=0, *maxcontemp=0;
  int contours = 0;
  contours = cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  int face=0;
  //最大轮廓dst
  for (contemp = contour;contemp != 0; contemp = contemp->h_next)  
  {
    face = fabs(cvContourArea(contemp));
    if(face > ConArea){
      ConArea = face;
      maxcontemp = contemp;
    }		
  }

  if(ConArea > 500)//如果面积太小说明没有找到纸片，可以不进行后续处理
  {
    ConExist = 1;		
    //多边形拟合dst1
    cont = cvApproxPoly(maxcontemp, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcontemp)*0.02, 0);//倒数第二个参数为拟合后周长误差，最后参数若为0，只处理src_seq指向的轮廓。1则处理整个双向链表中的所有轮廓。
    cvDrawContours (dst1, cont, cvScalar(255,0,0), cvScalar(255,0,0), 1, 4, 4);

    //找角点dst1	
    IplImage *eig_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1), *temp_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1); ;
    int maxcorners=4;

    cvGoodFeaturesToTrack(dst1, eig_image, temp_image, corners1, &maxcorners, 0.01, 150);//最后一个参数为两个点距离最小值	

    //角点顺时针排序，对于图片倾斜太多没办法dst1
    CvPoint2D32f cornertemp;
    int xy=corners1[0].x+corners1[0].y, maxxy=xy, minxy=xy, co2=0, co0=0;
    Center.y = corners1[0].y;
    Center.x = corners1[0].x;
    
    int i = 0;
    for( i = 1; i < maxcorners; i++ )  
    {
      Center.x += corners1[i].x;
      Center.y += corners1[i].y;
      xy=corners1[i].x+corners1[i].y;
      if(xy>maxxy){
        maxxy=xy;
        co2=i;
      }   
      else{
        if(xy<minxy){
          minxy=xy;
          co0=i;
        }
      }		
    }
    Center.x /= i;
    Center.y /= i;
    cvCircle( img, Center, 10, cvScalar(255,255,255),4);//在轮廓中心画圆

    if(co2!=2){
      cornertemp=corners1[co2];
      corners1[co2]=corners1[2];
      corners1[2]=cornertemp;
      if(co0==2){
        co0=co2;
      }
    }
    if(co0!=0){
      cornertemp=corners1[0];
      corners1[0]=corners1[co0];
      corners1[co0]=cornertemp;
    }
    if((corners1[1].x-corners1[0].x)<100){
      cornertemp=corners1[3];
      corners1[3]=corners1[1];
      corners1[1]=cornertemp;
    }


    //释放不需要的内存
    cvReleaseImage(&eig_image);
    cvReleaseImage(&temp_image);
  }

  //释放不需要的内存
  //cvReleaseImage(&src);
  cvReleaseImage(&dst1);
  cvReleaseImage(&dst);
  cvReleaseMemStorage (&storage);
  cvReleaseMemStorage (&storage1);
}

//数字判断函数
int ImgRecon::NumDetec(const IplImage* dst)
{
  int xsmin[10]={68,80,70,71,67,71,68,72,73,70},//样本图片数字的位置
      ysmin[10]={48,55,55,55,57,56,55,56,57,55},
      xsdel[10]={64,44,62,60,67,60,64,58,60,62},
      ysdel[10]={91,89,91,90,87,89,91,88,91,91};

  //框出目标数字部分
  int xmin=0,ymin=0,xmax=200,ymax=200,flag=0,i=0,j=0;
  unsigned char* p = (unsigned char*)(dst->imageData);

  for(i=50;i<100;++i)//col,width,x
  {
    for(j=35;j<130;++j)//row,height,y
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        xmin=i;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(j=35;j<100;++j)
  {
    for(i=50;i<150;++i)
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        ymin=j;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(i=140;i>100;--i)
  {
    for(j=150;j>70;--j)
    {
      if((int)p[i+j*dst->widthStep]==0)
      {
        xmax=i;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }

  flag=0;
  for(j=150;j>100;--j)
  {
    for(i=140;i>100;--i)
    {			
      if((int)p[i+j*dst->widthStep]==0)
      {
        ymax=j;
        flag=1;
        break;
      }
    }
    if(flag) break;
  }
  //cout<<"被检测图像数字位置："<<xmin<<"  "<<xmax<<"  "<<ymin<<"  "<<ymax<<endl;

  //判断数字
  int countdiff=0,k=0,mindiff=5000,tempdiff=0,rnum=-2;
  unsigned char* q;
  IplImage *sam;

  for(k=0;k<10;++k)
  {
    sam = cvLoadImage(SamLoca[k].c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if(!sam)
    {
      cout<<"Can not load sample image "<<k<<endl;
      return -1;
    }
    q = (unsigned char*)(sam->imageData);

    for(i=0;i<=xsdel[k];++i)
    {
      for(j=0;j<=ysdel[k];++j)
      {
        if((int)p[xmin+i+(j+ymin)*dst->widthStep]!=(int)q[xsmin[k]+i+(j+ysmin[k])*sam->widthStep])
          tempdiff++;
      }
    }
    //cout<<"与"<<k<<"的样本差别像素数为："<<tempdiff<<endl;
    if(tempdiff<mindiff) 
    {
      mindiff=tempdiff;
      rnum=k;
    }
    tempdiff = 0;
    cvReleaseImage(&sam);
  }
  if(mindiff>1000) return -1;
  return rnum;
}

//获得图片中的数字
int ImgRecon::GetNumber()
{
  if(NumResult == -2){
    IplImage *dst;
    dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1);
    //投影变换src->dst
    CvMat* warp_mat = cvCreateMat (3, 3, CV_32FC1);
    CvPoint2D32f dstTri[4];
    dstTri[0].x = 0;  
    dstTri[0].y = 0;  
    dstTri[1].x = 200;  
    dstTri[1].y = 0;  
    dstTri[2].x = 200;  
    dstTri[2].y = 200;  
    dstTri[3].x = 0;  
    dstTri[3].y = 200;
    cvGetPerspectiveTransform(corners1, dstTri, warp_mat);	
    cvWarpPerspective(src, dst, warp_mat);
    cvReleaseMat(&warp_mat);

    //判断数字
    NumResult = NumDetec(dst);
    cvReleaseImage(&dst);
  }
  return NumResult;
}

//获得轮廓中心
CvPoint ImgRecon::GetCenterPoint()
{
  return Center;
}

//获取轮廓面积
int ImgRecon::GetContourArea()
{
  return ConArea;
}

//轮廓存在与否
bool ImgRecon::ContourExist()
{
  return ConExist;
}
