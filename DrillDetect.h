#pragma once


#define _ONE_FRAME_POINT_COUNT_     700
#define _ROW_NUM_					1195
#define _COL_NUM_					1350
#define _ONE_FRAME_POINT_AVG_AREA_  500
#define	_FIRST_POINT_				450

// ��������
#define _FIRST_AREA_				0
#define _SECOND_AREA_				200
#define _THIRD_AREA_				400
#define _4TH_AREA_					450
#define	_5TH_AERA_					500
#define _ONCE_STEP_					200

// ex: ԭʼ��ά����Ϣ: �����ԭ���ݶ�ȡ��ת���ĵ�
struct _ORG_POINT_
{
	float* _intputPt_x;
	float* _intputPt_y;
	float* _intputPt_z;
};

// ex: �������ά����Ϣ
struct _Point_Calc_
{
	float  x;
	float  y;
	float  z;
};

// ex: ��֡��Ϣ
struct _Whole_Frame_Info_
{
	UINT* pt_realPoint;					// ��¼ÿһ֡����zֵ��������
	UINT* realStartIndex;				// ��ͷ��ʼ��һ��zֵ�������±꣨0-700��
	UINT* realEndIndex;					// ��ĩβ��ʼ��һ��zֵ�������±꣨0-700��
	UINT* lostPoint;					// ��¼ÿһ֡zֵΪ���ĸ���
	UINT* realPoint[4];					// ��¼ÿ֡���ĸ����� zֵ�����ĸ��� (0-150, 0-500, 200-700, 550-750)
	UINT* pt_Avg[4];					// ����ÿ֡���ĸ����� zֵƽ��ֵ
};

// ex: �����
struct _Neighbor_Calc_
{
	UINT * pt_Avg_less[4];				// ����x�ĸ����ֵ������ zֵ�þ�ֵ
	UINT * realLessPoint[4];			// �ĸ������������
};

// �˲�����
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
	// �㷨�ӿ�
	BOOL Detect(_ORG_POINT_* _Org_point, float threshold_Dis, float threshold_Frame, BOOL autoSave);

	// ������
	UINT m_pt_MaxDrump_Value;
	UINT m_pt_MaxDrump_X;
	UINT m_pt_MaxDrump_Y;
	UINT m_pt_MaxDrump_Z;
	float m_pct_LostFrame;
	
private:
	// �˲��㷨��
	BOOL FirstTraversal(float * pt_Input_X, float * pt_Input_Y, float * pt_Input_Z, _Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo);
	BOOL PtNeighborCalc(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc);
	BOOL SecondTravFite(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc, _FITE_PARA_ *_filePara, float threshold_Dis, float threshold_Frame);
	BOOL WaveFilter(int pointX, int pointY, float a, float b, BOOL lostFrame);
	BOOL CalcResult(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, UINT GiveUpPoints);
	BOOL FreeMemory(_Point_Calc_ *_point, _Whole_Frame_Info_ *_wholeFrameInfo, _Neighbor_Calc_ *_neighbor_Calc, _FITE_PARA_ *_filePara);
	BOOL AutoSavePly(_Point_Calc_ *_point);
	char* GetTimeForCur();
	
};

