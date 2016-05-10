// ImgRecon.cpp: implementation of the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ImgRecon.h"
#include <iostream>
using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ImgRecon::ImgRecon(const IplImage *img)
{
	
	ReInit(img);
}

ImgRecon::~ImgRecon()
{

}

//更改下一张图片
void ImgRecon::ReInit(const IplImage *img)
{	
	NumResult = -2;
	Center = cvPoint(0,0);
	ConArea = 0;
	ConExist = 0;
	SamLoca = "F:\\photo\\numbers\\c1.jpg";//样本存放地址
	
	IplImage 
		*src,//灰度图，二值化，腐蚀
		*dst,//复制src后找轮廓
		*dst1;//画拟合曲线，找角点
	src = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
	dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
	dst1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);

	cvCvtColor( img, src, CV_BGR2GRAY );//转灰img->src
	cvThreshold( src, src, 148, 255, CV_THRESH_BINARY);//二值化src
	cvErode( src, src, NULL, 2);//腐蚀src
	cvCopy( src, dst);//备份src->dst

	//找轮廓dst
	CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
	CvSeq *contour = 0, *cont=0, *contemp=0, *maxcontemp=0;
	int contours = 0;
	contours = cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	int face=0, ConArea=0;
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
		NumResult = -1;
		cout<<"ConArea="<<ConArea<<endl;
		//多边形拟合dst1
		cont = cvApproxPoly(maxcontemp, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcontemp)*0.02, 0);//倒数第二个参数为拟合后周长误差，最后参数若为0，只处理src_seq指向的轮廓。1则处理整个双向链表中的所有轮廓。
		cvDrawContours (dst1, cont, cvScalar(255,0,0), cvScalar(255,0,0), 1, 4, 4);

		//找角点dst1	
		IplImage *eig_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1), *temp_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1); ;
		int maxcorners=4;
		CvPoint2D32f corners1[4];//
		cvGoodFeaturesToTrack(dst1, eig_image, temp_image, corners1, &maxcorners, 0.01, 300);//最后一个参数为两个点距离最小值	
	
		//角点顺时针排序，对于c1图片倾斜太多没办法dst1
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
		if((corners1[1].x-corners1[0].x)<250){
			cornertemp=corners1[3];
			corners1[3]=corners1[1];
			corners1[1]=cornertemp;
		}

		//投影变换src->dst
		CvMat* warp_mat = cvCreateMat (3, 3, CV_32FC1);
		CvPoint2D32f dstTri[4];
		dstTri[0].x = 0;  
	    dstTri[0].y = 0;  
		dstTri[1].x = 500;  
	    dstTri[1].y = 0;  
		dstTri[2].x = 500;  
	    dstTri[2].y = 500;  
		dstTri[3].x = 0;  
		dstTri[3].y = 500;
		cvGetPerspectiveTransform(corners1, dstTri, warp_mat);	
		cvWarpPerspective(src, dst, warp_mat);

		//释放不需要的内存
		cvReleaseImage(&eig_image);
		cvReleaseImage(&temp_image);
	}

	//释放不需要的内存
	cvReleaseImage(&dst);
	cvReleaseImage(&dst1);
	cvReleaseImage(&src);
	cvReleaseMemStorage (&storage);
	cvReleaseMemStorage (&storage1);
	cout<<"ConArea1="<<ConArea<<endl;
}

//获得图片中的数字
int ImgRecon::GetNumber()
{
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
	cout<<"ConArea2="<<ConArea<<endl;
	return ConArea;
}

//轮廓存在与否
bool ImgRecon::ContourExist()
{
	return ConExist;
}