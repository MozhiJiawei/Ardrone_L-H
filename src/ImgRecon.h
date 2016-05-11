// ImgRecon.h: interface for the ImgRecon class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
#define AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <highgui.h>
#include <cv.h>
#include <string>
using namespace std;
class ImgRecon  
{
public:
	ImgRecon(const IplImage *img);//����ֱ�Ӵ���ͼ
	virtual ~ImgRecon();
	void ReInit(const IplImage *img);//������һ��ͼƬ
	int GetNumber();//���ͼƬ�е�����
	CvPoint GetCenterPoint();//�����������
	int GetContourArea();//��ȡ�������
	bool ContourExist();//�����������

private:
	int NumResult;//�ж���������
	bool ConExist;//�����Ƿ����
	int ConArea;//�������
	CvPoint Center;//�������ĵ�
	string SamLoca[10];//����λ��
	int NumDetec(const IplImage* dst);//�����жϺ���
	//IplImage 
	//	*src,//�Ҷ�ͼ����ֵ������ʴ
	//	*dst,//����src��������
	//	*dst1;//��������ߣ��ҽǵ�
	
};

#endif // !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
