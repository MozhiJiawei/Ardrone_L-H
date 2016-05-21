// ImgRecon.h: interface for the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
#define AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <math.h>
#include <highgui.h>
#include <cv.h>
#include <string>
using namespace std;
class ImgRecon  
{
  public:
    ImgRecon(IplImage *img);//give a colorful picture
    virtual ~ImgRecon();
    void ReInit(IplImage *img);//change picture
    int GetNumber();//get the number in the picture,-1 is not recon
    CvPoint GetCenterPoint();//get contour center
    int GetContourArea();//get contour area
    bool ContourExist();//is contour exist?
    float GetTopPointDiff();//>0,left higher;<0,right higher;return arcsin

  private:
    int NumResult;//�ж���������
    bool ConExist;//�����Ƿ����
    int ConArea;//�������
    CvPoint Center;//�������ĵ�
    CvPoint2D32f corners1[4];//�ĸ�˳ʱ�����еĽǵ㣬��һ��Ϊ����
    string SamLoca[10];//����λ��
    int NumDetec(const IplImage* dst);//�����жϺ���
    IplImage 
      *src;//�Ҷ�ͼ����ֵ������ʴ
    //	*dst;//����src������������ű�׼�����ͼƬ
    //	*dst1;//��������ߣ��ҽǵ�

};

#endif // !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
