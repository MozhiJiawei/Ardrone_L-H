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
	ImgRecon(const IplImage *img);//可以直接传彩图
	virtual ~ImgRecon();
	void ReInit(const IplImage *img);//更改下一张图片
	int GetNumber();//获得图片中的数字
	CvPoint GetCenterPoint();//获得轮廓中心
	int GetContourArea();//获取轮廓面积
	bool ContourExist();//轮廓存在与否

private:
	int NumResult;//判断所得数字
	bool ConExist;//轮廓是否存在
	int ConArea;//轮廓面积
	CvPoint Center;//轮廓中心点
	string SamLoca[10];//样本位置
	int NumDetec(const IplImage* dst);//数字判断函数
	//IplImage 
	//	*src,//灰度图，二值化，腐蚀
	//	*dst,//复制src后找轮廓
	//	*dst1;//画拟合曲线，找角点
	
};

#endif // !defined(AFX_IMGRECON_H__A74914C1_139A_4D1D_B134_668BD20B3AF8__INCLUDED_)
