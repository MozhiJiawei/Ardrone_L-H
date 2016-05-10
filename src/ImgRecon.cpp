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

//������һ��ͼƬ
void ImgRecon::ReInit(const IplImage *img)
{	
	NumResult = -2;
	Center = cvPoint(0,0);
	ConArea = 0;
	ConExist = 0;
	SamLoca = "F:\\photo\\numbers\\c1.jpg";//������ŵ�ַ
	
	IplImage 
		*src,//�Ҷ�ͼ����ֵ������ʴ
		*dst,//����src��������
		*dst1;//��������ߣ��ҽǵ�
	src = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
	dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
	dst1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);

	cvCvtColor( img, src, CV_BGR2GRAY );//ת��img->src
	cvThreshold( src, src, 148, 255, CV_THRESH_BINARY);//��ֵ��src
	cvErode( src, src, NULL, 2);//��ʴsrc
	cvCopy( src, dst);//����src->dst

	//������dst
	CvMemStorage *storage = cvCreateMemStorage(0), *storage1 = cvCreateMemStorage(0);
	CvSeq *contour = 0, *cont=0, *contemp=0, *maxcontemp=0;
	int contours = 0;
	contours = cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	int face=0, ConArea=0;
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
		NumResult = -1;
		cout<<"ConArea="<<ConArea<<endl;
		//��������dst1
		cont = cvApproxPoly(maxcontemp, sizeof(CvContour), storage1, CV_POLY_APPROX_DP, cvContourPerimeter(maxcontemp)*0.02, 0);//�����ڶ�������Ϊ��Ϻ��ܳ�����������Ϊ0��ֻ����src_seqָ���������1��������˫�������е�����������
		cvDrawContours (dst1, cont, cvScalar(255,0,0), cvScalar(255,0,0), 1, 4, 4);

		//�ҽǵ�dst1	
		IplImage *eig_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1), *temp_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1); ;
		int maxcorners=4;
		CvPoint2D32f corners1[4];//
		cvGoodFeaturesToTrack(dst1, eig_image, temp_image, corners1, &maxcorners, 0.01, 300);//���һ������Ϊ�����������Сֵ	
	
		//�ǵ�˳ʱ�����򣬶���c1ͼƬ��б̫��û�취dst1
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

		//ͶӰ�任src->dst
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

		//�ͷŲ���Ҫ���ڴ�
		cvReleaseImage(&eig_image);
		cvReleaseImage(&temp_image);
	}

	//�ͷŲ���Ҫ���ڴ�
	cvReleaseImage(&dst);
	cvReleaseImage(&dst1);
	cvReleaseImage(&src);
	cvReleaseMemStorage (&storage);
	cvReleaseMemStorage (&storage1);
	cout<<"ConArea1="<<ConArea<<endl;
}

//���ͼƬ�е�����
int ImgRecon::GetNumber()
{
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
	cout<<"ConArea2="<<ConArea<<endl;
	return ConArea;
}

//�����������
bool ImgRecon::ContourExist()
{
	return ConExist;
}