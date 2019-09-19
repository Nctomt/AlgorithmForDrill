#pragma once


#define _ONE_FRAME_POINT_COUNT_     700
#define _ROW_NUM_					1195
#define _COL_NUM_					1350
#define _ONE_FRAME_POINT_AVG_AREA_  500
#define	_FIRST_POINT_				450

// 监测区域宏
#define _FIRST_AREA_				0
#define _SECOND_AREA_				200
#define _THIRD_AREA_				400
#define _4TH_AREA_					450
#define	_5TH_AERA_					500
#define _ONCE_STEP_					200

// ex: 原始三维点信息: 相机从原数据读取来转换的点
struct _ORG_POINT_
{
	float* _intputPt_x;
	float* _intputPt_y;
	float* _intputPt_z;
};

// ex: 处理后三维点信息
struct _Point_Calc_
{
	float  x;
	float  y;
	float  z;
};

// ex: 整帧信息
struct _Whole_Frame_Info_
{
	UINT* pt_realPoint;					// 记录每一帧所有z值非零点个数
	UINT* realStartIndex;				// 从头开始第一个z值非零点的下标（0-700）
	UINT* realEndIndex;					// 从末尾开始第一个z值非零点的下标（0-700）
	UINT* lostPoint;					// 记录每一帧z值为零点的个数
	UINT* realPoint[4];					// 记录每帧中四个区域 z值非零点的个数 (0-150, 0-500, 200-700, 550-750)
	UINT* pt_Avg[4];					// 计算每帧中四个区域 z值平均值
};

// ex: 邻域点
struct _Neighbor_Calc_
{
	UINT * pt_Avg_less[4];				// 计算x四个均分点的邻域 z值得均值
	UINT * realLessPoint[4];			// 四个邻域非零点的数
};

// 滤波参数
struct _FITE_PARA_
{
	float * point_X[4];
	float * point_Z[4] ;
	float * a[3];
	float * b[3];
	UINT * distance;
};
class CDrillDetect
{
public:
	CDrillDetect();
	~CDrillDetect();
public:
	// 算法接口
	BOOL Detect(_ORG_POINT_* _Org_point, float threshold_Dis, float threshold_Frame, BOOL autoSave);

	// 计算结果
	UINT m_pt_MaxDrump_Value;
	UINT m_pt_MaxDrump_X;
	UINT m_pt_MaxDrump_Y;
	UINT m_pt_MaxDrump_Z;
	float m_pct_LostFrame;
	
private:
	// 滤波算法：
	BOOL FirstTraversal(float * pt_Input_X, float * pt_Input_Y, float * pt_Input_Z, _Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo);
	BOOL PtNeighborCalc(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc);
	BOOL SecondTravFite(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc, _FITE_PARA_ *_filePara, float threshold_Dis, float threshold_Frame);
	BOOL WaveFilter(int pointX, int pointY, float a, float b, BOOL lostFrame);
	BOOL CalcResult(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, UINT GiveUpPoints);
	BOOL FreeMemory(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc, _FITE_PARA_ *_filePara);
	BOOL AutoSavePly(_Point_Calc_ *_point);
	char* GetTimeForCur();
	
};

