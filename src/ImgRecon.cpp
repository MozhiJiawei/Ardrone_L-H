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
  SamLoca[0]="samples\\sample0.bmp";SamLoca[5]="samples\\sample5.bmp";//������ŵ�ַ
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

//������һ��ͼƬ
void ImgRecon::ReInit(IplImage *img)
{	
  NumResult = -2;
  Center = cvPoint(320,180);
  ConArea = 0;
  ConExist = 0;


  IplImage 
    //*src,//�Ҷ�ͼ����ֵ������ʴ
    *dst,//����src��������
    *dst1;//��������ߣ��ҽǵ�
  //src = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  dst1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);

  cvCvtColor( img, src, CV_BGR2GRAY );//ת��img->src
  cvThreshold( src, src, 130, 255, CV_THRESH_BINARY);//��ֵ��src
  cvErode( src, src, NULL, 1);//��ʴsrc
  cvCopy( src, dst);//����src->dst

  //������dst
  CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
  CvSeq *contour = 0, *cont=0, *contemp=0, *maxcontemp=0;
  int contours = 0;
  contours = cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  int face=0;
  //�������dst
  for (contemp = contour;contemp != 0; contemp = contemp->h_next)  
  {
    face = fabs(cvContourArea(contemp));
    if(face > ConArea){
      ConArea = face;
      maxcontemp = contemp;
    }		
  }

  if(ConArea > 500)//������̫С˵��û���ҵ�ֽƬ�����Բ����к�������
  {
    ConExist = 1;		
    //��������dst1
    cont = cvApproxPoly(maxcontemp, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcontemp)*0.02, 0);//�����ڶ�������Ϊ��Ϻ��ܳ�����������Ϊ0��ֻ����src_seqָ���������1��������˫�������е�����������
    cvDrawContours (dst1, cont, cvScalar(255,0,0), cvScalar(255,0,0), 1, 4, 4);

    //�ҽǵ�dst1	
    IplImage *eig_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1), *temp_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1); ;
    int maxcorners=4;

    cvGoodFeaturesToTrack(dst1, eig_image, temp_image, corners1, &maxcorners, 0.01, 150);//���һ������Ϊ�����������Сֵ	

    //�ǵ�˳ʱ�����򣬶���ͼƬ��б̫��û�취dst1
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
    cvCircle( img, Center, 10, cvScalar(255,255,255),4);//���������Ļ�Բ

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


    //�ͷŲ���Ҫ���ڴ�
    cvReleaseImage(&eig_image);
    cvReleaseImage(&temp_image);
  }

  //�ͷŲ���Ҫ���ڴ�
  //cvReleaseImage(&src);
  cvReleaseImage(&dst1);
  cvReleaseImage(&dst);
  cvReleaseMemStorage (&storage);
  cvReleaseMemStorage (&storage1);
}

//�����жϺ���
int ImgRecon::NumDetec(const IplImage* dst)
{
  int xsmin[10]={68,80,70,71,67,71,68,72,73,70},//����ͼƬ���ֵ�λ��
      ysmin[10]={48,55,55,55,57,56,55,56,57,55},
      xsdel[10]={64,44,62,60,67,60,64,58,60,62},
      ysdel[10]={91,89,91,90,87,89,91,88,91,91};

  //���Ŀ�����ֲ���
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
  //cout<<"�����ͼ������λ�ã�"<<xmin<<"  "<<xmax<<"  "<<ymin<<"  "<<ymax<<endl;

  //�ж�����
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
    //cout<<"��"<<k<<"���������������Ϊ��"<<tempdiff<<endl;
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

//���ͼƬ�е�����
int ImgRecon::GetNumber()
{
  if(NumResult == -2){
    IplImage *dst;
    dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1);
    //ͶӰ�任src->dst
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

    //�ж�����
    NumResult = NumDetec(dst);
    cvReleaseImage(&dst);
  }
  return NumResult;
}

//�����������
CvPoint ImgRecon::GetCenterPoint()
{
  return Center;
}

//��ȡ�������
int ImgRecon::GetContourArea()
{
  return ConArea;
}

//�����������
bool ImgRecon::ContourExist()
{
  return ConExist;
}
