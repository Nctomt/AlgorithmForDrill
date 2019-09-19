#include "stdafx.h"
#include "DrillDetect.h"


CDrillDetect::CDrillDetect()
	: m_pt_MaxDrump_Value(0)
	, m_pt_MaxDrump_X(0)
	, m_pt_MaxDrump_Y(0)
	, m_pt_MaxDrump_Z(0)
	, m_pct_LostFrame(0.0f)
{
}


CDrillDetect::~CDrillDetect()
{
}

// FUC：第一次遍历	
//		1: 截取计算区域(45000 - 11500), 每帧700个点
//		2: 把 x / 10, 使整体走势及凑，方便拟合
//		3: 找出每一帧的正数/倒数第一个点
//		4: 求出多个区域的均值 用来做y值点
BOOL CDrillDetect::FirstTraversal(float * pt_Input_X, float * pt_Input_Y, float * pt_Input_Z, _Point_Calc_ *_point, _Whole_Frame_Info_ *wholeFrameInfo)
{
	if (NULL == pt_Input_X || NULL == pt_Input_Y || NULL == pt_Input_Z || NULL == _point || NULL == wholeFrameInfo)
	{
		return FALSE;
	}

	for (size_t i = 0; i < 4; i++) {
		wholeFrameInfo->pt_Avg[i] = new UINT[_ROW_NUM_];
		wholeFrameInfo->realPoint[i] = new UINT[_ROW_NUM_];
		memset(wholeFrameInfo->pt_Avg[i], 0, sizeof(UINT) * _ROW_NUM_);
		memset(wholeFrameInfo->realPoint[i], 0, sizeof(UINT) * _ROW_NUM_);
	}
	wholeFrameInfo->lostPoint = new UINT[_ROW_NUM_];
	wholeFrameInfo->pt_realPoint = new UINT[_ROW_NUM_];
	wholeFrameInfo->realStartIndex = new UINT[_ROW_NUM_];
	wholeFrameInfo->realEndIndex = new UINT[_ROW_NUM_];
	memset(wholeFrameInfo->lostPoint, 0, sizeof(UINT) * _ROW_NUM_);
	memset(wholeFrameInfo->pt_realPoint, 0, sizeof(UINT) * _ROW_NUM_);
	memset(wholeFrameInfo->realStartIndex, 0, sizeof(UINT) * _ROW_NUM_);
	memset(wholeFrameInfo->realEndIndex, 0, sizeof(UINT) * _ROW_NUM_);
	
	
	BOOL Flag = TRUE;								// 用于标定每帧正数/倒数第一个点
	UINT pointNum = _FIRST_POINT_;					// 开始第一个点
													
	for (size_t j = 0; j < _ROW_NUM_; j++)
	{
		Flag = TRUE;
		for (size_t i = 0; i < _ONE_FRAME_POINT_COUNT_; i++)
		{
			_point[i + j * _ONE_FRAME_POINT_COUNT_].x = pt_Input_X[pointNum] / 10;
			_point[i + j * _ONE_FRAME_POINT_COUNT_].y = pt_Input_Y[pointNum];
			_point[i + j * _ONE_FRAME_POINT_COUNT_].z = pt_Input_Z[pointNum];
			pointNum++;
			// 计算每帧缺点数
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z == 0)
			{
				wholeFrameInfo->lostPoint[j]++;
			}
			// 计算出多区域均值
			if ((i >= 0 && i < 150) && !((_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6)) {
				wholeFrameInfo->pt_Avg[0][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				wholeFrameInfo->realPoint[0][j]++;
			}
			if ((i >= 0 && i < 500) && !((_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6)) {
				wholeFrameInfo->pt_Avg[1][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				wholeFrameInfo->realPoint[1][j]++;
			}
			if ((i >= 200 && i < 700) && !((_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6)) {
				wholeFrameInfo->pt_Avg[2][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				wholeFrameInfo->realPoint[2][j]++;
			}
			if ((i >= 550 && i < 700) && !((_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6)) {
				wholeFrameInfo->pt_Avg[3][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				wholeFrameInfo->realPoint[3][j]++;
			}
			// 找出正数/倒数第一个点
			if (!((_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6))
			{
				wholeFrameInfo->pt_realPoint[j]++;
				if (Flag)
				{
					wholeFrameInfo->realStartIndex[j] = i;
					Flag = FALSE;
				}
				wholeFrameInfo->realEndIndex[j] = i;
			}
		}
		pointNum += (_COL_NUM_ - _ONE_FRAME_POINT_COUNT_);
	}

	// 计算每一帧均值 
	for (size_t i = 0; i < _ROW_NUM_; i++)
	{
		wholeFrameInfo->pt_Avg[0][i] = (wholeFrameInfo->realPoint[0][i] == 0) ? 0 : (wholeFrameInfo->pt_Avg[0][i] / wholeFrameInfo->realPoint[0][i]);
		wholeFrameInfo->pt_Avg[1][i] = (wholeFrameInfo->realPoint[1][i] == 0) ? 0 : (wholeFrameInfo->pt_Avg[1][i] / wholeFrameInfo->realPoint[1][i]);
		wholeFrameInfo->pt_Avg[2][i] = (wholeFrameInfo->realPoint[2][i] == 0) ? 0 : (wholeFrameInfo->pt_Avg[2][i] / wholeFrameInfo->realPoint[2][i]);
		wholeFrameInfo->pt_Avg[3][i] = (wholeFrameInfo->realPoint[3][i] == 0) ? 0 : (wholeFrameInfo->pt_Avg[3][i] / wholeFrameInfo->realPoint[3][i]);
	}
	return TRUE;
}

// FUC：求出四个点邻域的均值	
//		x方向分别为 正数/倒数的第一个点， 1/3, 2/3 四个点  附近50个点的均值
//		由于中间两个点为固定区域，可能邻域全为零
//		加入异常判断：如果为零 则用第一次计算算出来的大区域进行替换
BOOL CDrillDetect::PtNeighborCalc(_Point_Calc_ * _point, _Whole_Frame_Info_ * _wholeFrameInfo, _Neighbor_Calc_ * _neighbor_Calc)
{
	if (NULL == _point || NULL == _wholeFrameInfo || NULL == _neighbor_Calc)
	{
		return FALSE;
	}

	for (size_t i = 0; i < 4; i++)
	{
		_neighbor_Calc->pt_Avg_less[i] = new UINT[_ROW_NUM_];
		_neighbor_Calc->realLessPoint[i] = new UINT[_ROW_NUM_];
		memset(_neighbor_Calc->pt_Avg_less[i], 0, sizeof(UINT) * _ROW_NUM_);
		memset(_neighbor_Calc->realLessPoint[i], 0, sizeof(UINT) * _ROW_NUM_);
	}


	// test： 四个点选取  节点附近10个点的均值
	for (size_t j = 0; j < _ROW_NUM_; j++)
	{
		for (size_t i = _wholeFrameInfo->realStartIndex[j]; i < _wholeFrameInfo->realStartIndex[j] + 50; i++)
		{
			if (!(_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0 < 1e-6))
			{
				_neighbor_Calc->pt_Avg_less[0][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				_neighbor_Calc->realLessPoint[0][j]++;
			}
		}
		for (size_t i = _ONE_FRAME_POINT_COUNT_ / 3; i < _ONE_FRAME_POINT_COUNT_ / 3 + 50; i++)
		{
			if (!(_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0 < 1e-6))
			{
				_neighbor_Calc->pt_Avg_less[1][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				_neighbor_Calc->realLessPoint[1][j]++;
			}
		}
		for (size_t i = _ONE_FRAME_POINT_COUNT_ / 3 * 2; i < _ONE_FRAME_POINT_COUNT_ / 3 * 2 + 50; i++)
		{
			if (!(_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0 < 1e-6))
			{
				_neighbor_Calc->pt_Avg_less[2][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				_neighbor_Calc->realLessPoint[2][j]++;
			}
		}
		for (size_t i = _wholeFrameInfo->realEndIndex[j]; i > _wholeFrameInfo->realEndIndex[j] - 50; i--)
		{
			if (!(_point[i + j * _ONE_FRAME_POINT_COUNT_].z - 0.0 < 1e-6))
			{
				_neighbor_Calc->pt_Avg_less[3][j] += _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				_neighbor_Calc->realLessPoint[3][j]++;
			}
		}
	}
	for (size_t i = 0; i < _ROW_NUM_; i++)
	{
		_neighbor_Calc->pt_Avg_less[0][i] = (_neighbor_Calc->realLessPoint[0][i] == 0) ? _wholeFrameInfo->pt_Avg[0][i] : (_neighbor_Calc->pt_Avg_less[0][i] / _neighbor_Calc->realLessPoint[0][i]);
		_neighbor_Calc->pt_Avg_less[1][i] = (_neighbor_Calc->realLessPoint[1][i] == 0) ? _wholeFrameInfo->pt_Avg[1][i] : (_neighbor_Calc->pt_Avg_less[1][i] / _neighbor_Calc->realLessPoint[1][i]);
		_neighbor_Calc->pt_Avg_less[2][i] = (_neighbor_Calc->realLessPoint[2][i] == 0) ? _wholeFrameInfo->pt_Avg[2][i] : (_neighbor_Calc->pt_Avg_less[2][i] / _neighbor_Calc->realLessPoint[2][i]);
		_neighbor_Calc->pt_Avg_less[3][i] = (_neighbor_Calc->realLessPoint[3][i] == 0) ? _wholeFrameInfo->pt_Avg[3][i] : (_neighbor_Calc->pt_Avg_less[3][i] / _neighbor_Calc->realLessPoint[3][i]);
	}
	return TRUE;
}

// FUC: 第二次遍历：滤波
//		四个点确定三条直线的线性方程：需要加入临界区域判定 如果恰好为零 向后找第一个不为零的点
//		计算每个点的点到直线距离，通过距离阈值的设定进行滤波
BOOL CDrillDetect::SecondTravFite(_Point_Calc_ * _point, _Whole_Frame_Info_ * _wholeFrameInfo, _Neighbor_Calc_ * _neighbor_Calc, _FITE_PARA_ * _filePara, float threshold_Dis, float threshold_Frame)
{
	if (NULL == _point || NULL == _wholeFrameInfo || NULL == _neighbor_Calc || NULL == _filePara)
	{
		return FALSE;
	}
	for (size_t i = 0; i < 4; i++)
	{
		_filePara->point_X[i] = new float[_ROW_NUM_];
		_filePara->point_Z[i] = new float[_ROW_NUM_];
		memset(_filePara->point_X[i], 0, sizeof(float) * _ROW_NUM_);
		memset(_filePara->point_Z[i], 0, sizeof(float) * _ROW_NUM_);
	}
	for (size_t i = 0; i < 3; i++)
	{
		_filePara->a[i] = new float[_ROW_NUM_];
		_filePara->b[i] = new float[_ROW_NUM_];
		memset(_filePara->a[i], 0, sizeof(float) * _ROW_NUM_);
		memset(_filePara->b[i], 0, sizeof(float) * _ROW_NUM_);
	}
	_filePara->distance = new UINT[_ROW_NUM_];
	memset(_filePara->distance, 0, sizeof(UINT) * _ROW_NUM_);


	// 计算每一帧的四个点
	for (size_t i = 0; i < _ROW_NUM_; i++)
	{
		// point 1 : x为最左侧点，y为区域一均值（0-150）
		_filePara->point_X[0][i] = _point[_wholeFrameInfo->realStartIndex[i] + i * _ONE_FRAME_POINT_COUNT_].x;
		_filePara->point_Z[0][i] = _neighbor_Calc->pt_Avg_less[0][i];

		// point 2 : x为1/3开始右侧第一个不为零得点 ，y为区域一均值（0-500）
		_filePara->point_X[1][i] = _point[(_wholeFrameInfo->pt_realPoint[i] / 3) + i * _ONE_FRAME_POINT_COUNT_].x;
	//	_filePara->point_Z_2[i] = _neighbor_Calc->pt_Avg_less1[i];
		_filePara->point_Z[1][i] = _wholeFrameInfo->pt_Avg[1][i];
		/// 如果交界点未零 则向后选取第一个不为零的点
		if (_filePara->point_Z[1][i] - 0.0 < 1e-6)
		{
			for (int j = _wholeFrameInfo->pt_realPoint[i] / 3; j < _wholeFrameInfo->pt_realPoint[i]; j++)
			{
				if (!(_point[j + i * _ONE_FRAME_POINT_COUNT_].z - 0.0f < 1e-6)) {
					_filePara->point_X[1][i] = _point[j + i * _ONE_FRAME_POINT_COUNT_].x;
					_filePara->point_Z[1][i] = _point[j + i * _ONE_FRAME_POINT_COUNT_].z;
					break;
				}
			}
		}
		// point 3 : x为2/3开始左侧第一个不为零得点，y为区域一均值（200-500）
		_filePara->point_X[2][i] = _point[(_wholeFrameInfo->pt_realPoint[i] / 3 * 2) + i * _ONE_FRAME_POINT_COUNT_].x;
	//	_filePara->point_Z_3[i] = _neighbor_Calc->pt_Avg_less2[i];
		_filePara->point_Z[2][i] = _wholeFrameInfo->pt_Avg[2][i];
		if (_filePara->point_Z[2][i] - 0.0 < 1e-6)
		{
			for (int j = _wholeFrameInfo->pt_realPoint[i] / 3 * 2; j > _wholeFrameInfo->realStartIndex[i]; j--)
			{
				if (!(_point[j + i * _ONE_FRAME_POINT_COUNT_].z - 0.0f < 1e-6)) {
					_filePara->point_X[2][i] = _point[j + i * _ONE_FRAME_POINT_COUNT_].x;
					_filePara->point_Z[2][i] = _point[j + i * _ONE_FRAME_POINT_COUNT_].z;
					break;
				}
			}
		}
		// point 4 : x为最右侧点，y为区域一均值（550-700）
		_filePara->point_X[3][i] = _point[_wholeFrameInfo->realEndIndex[i] + i * _ONE_FRAME_POINT_COUNT_].x;
		_filePara->point_Z[3][i] = _neighbor_Calc->pt_Avg_less[3][i];
	}

	// 计算三条线的公式
	/// 计算 y = ax + b的 a和b
	for (size_t i = 0; i < _ROW_NUM_; i++)
	{
		_filePara->a[0][i] = (_filePara->point_Z[1][i] - _filePara->point_Z[0][i]) / (_filePara->point_X[1][i] - _filePara->point_X[0][i]);
		_filePara->b[0][i] = _filePara->point_Z[0][i] - _filePara->a[0][i] * _filePara->point_X[0][i];

		_filePara->a[1][i] = (_filePara->point_Z[2][i] - _filePara->point_Z[1][i]) / (_filePara->point_X[2][i] - _filePara->point_X[1][i]);
		_filePara->b[1][i] = _filePara->point_Z[1][i] - _filePara->a[1][i] * _filePara->point_X[1][i];

		_filePara->a[2][i] = (_filePara->point_Z[3][i] - _filePara->point_Z[2][i]) / (_filePara->point_X[3][i] - _filePara->point_X[2][i]);
		_filePara->b[2][i] = _filePara->point_Z[2][i] - _filePara->a[2][i] * _filePara->point_X[2][i];
		_filePara->distance[i] = _wholeFrameInfo->pt_realPoint[i] / 3;
	}
	//UpdateData(TRUE);
	/// 画线以实际存在点为总体 分成三段
	BOOL thresholdPlus = FALSE;			// 大于阈值 TRUE  小于阈值 FALSE
	for (size_t j = 0; j < _ROW_NUM_; j++)
	{
		thresholdPlus = _wholeFrameInfo->lostPoint[j] > threshold_Frame ? TRUE : FALSE;
		for (size_t i = 0; i < _filePara->distance[j]; i++)
		{
			if (!((_point[i + _filePara->distance[j] * 0 + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6) && !WaveFilter(_point[i + _filePara->distance[j] * 0 + j * _ONE_FRAME_POINT_COUNT_].x, _point[i + _filePara->distance[j] * 0 + j * _ONE_FRAME_POINT_COUNT_].z, _filePara->a[0][j], _filePara->b[0][j], thresholdPlus))
			{
				_point[i + _filePara->distance[j] * 0 + j * _ONE_FRAME_POINT_COUNT_].z = 0;
			}
			if (!((_point[i + _filePara->distance[j] * 1 + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6) && !WaveFilter(_point[i + _filePara->distance[j] * 1 + j * _ONE_FRAME_POINT_COUNT_].x, _point[i + _filePara->distance[j] * 1 + j * _ONE_FRAME_POINT_COUNT_].z, _filePara->a[1][j], _filePara->b[1][j], thresholdPlus))
			{
				_point[i + _filePara->distance[j] * 1 + j * _ONE_FRAME_POINT_COUNT_].z = 0;
			}
			if (!((_point[i + _filePara->distance[j] * 2 + j * _ONE_FRAME_POINT_COUNT_].z - 0.0) < 1e-6) && !WaveFilter(_point[i + _filePara->distance[j] * 2 + j * _ONE_FRAME_POINT_COUNT_].x, _point[i + _filePara->distance[j] * 2 + j * _ONE_FRAME_POINT_COUNT_].z, _filePara->a[2][j], _filePara->b[2][j], thresholdPlus))
			{
				_point[i + _filePara->distance[j] * 2 + j * _ONE_FRAME_POINT_COUNT_].z = 0;
			}
		}
	}
	return TRUE;
}

// FUC: 第三次遍历：计算
//		五个区域分别 0-200  200-400  400-600  450-650  500-700 沿Y方向划分的区域
//		对指定区域进行物理宽度2cm的径向检测
//		计算出最大跳动公差，以及最大点对应的xyz值和总体缺帧率（零的点数/总点数）
BOOL CDrillDetect::CalcResult(_Point_Calc_ * _point, _Whole_Frame_Info_ *_wholeFrameInfo, UINT GiveUpPoints)
{
	if (NULL == _point || NULL == _wholeFrameInfo)
	{
		return FALSE;
	}
	m_pt_MaxDrump_Value = 0;
	m_pt_MaxDrump_X = 0;
	m_pt_MaxDrump_Y = 0;
	m_pt_MaxDrump_Z = 0;
	m_pct_LostFrame = 0.0f;

	// 计算五次 五个区域
	UINT Area1_MAX_Point_Z[_ROW_NUM_] = { 0 };
	UINT Area2_MAX_Point_Z[_ROW_NUM_] = { 0 };
	UINT Area3_MAX_Point_Z[_ROW_NUM_] = { 0 };
	UINT Area4_MAX_Point_Z[_ROW_NUM_] = { 0 };
	UINT Area5_MAX_Point_Z[_ROW_NUM_] = { 0 };

	UINT Area1_Index[_ROW_NUM_] = { 0 };
	UINT Area2_Index[_ROW_NUM_] = { 0 };
	UINT Area3_Index[_ROW_NUM_] = { 0 };
	UINT Area4_Index[_ROW_NUM_] = { 0 };
	UINT Area5_Index[_ROW_NUM_] = { 0 };
	// 计算出每帧极大值点以及坐标
	for (size_t j = 0; j < _ROW_NUM_; j++)
	{
		for (size_t i = _FIRST_AREA_; i < _FIRST_AREA_ + _ONCE_STEP_; i++)
		{
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z > Area1_MAX_Point_Z[j]) {
				Area1_MAX_Point_Z[j] = _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				Area1_Index[j] = i + j * _ONE_FRAME_POINT_COUNT_;
			}
		}
		for (size_t i = _SECOND_AREA_; i < _SECOND_AREA_ + _ONCE_STEP_; i++)
		{
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z > Area2_MAX_Point_Z[j]) {
				Area2_MAX_Point_Z[j] = _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				Area2_Index[j] = i + j * _ONE_FRAME_POINT_COUNT_;
			}
		}
		for (size_t i = _THIRD_AREA_; i < _THIRD_AREA_ + _ONCE_STEP_; i++)
		{
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z > Area3_MAX_Point_Z[j]) {
				Area3_MAX_Point_Z[j] = _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				Area3_Index[j] = i + j * _ONE_FRAME_POINT_COUNT_;
			}
		}
		for (size_t i = _4TH_AREA_; i < _4TH_AREA_ + _ONCE_STEP_; i++)
		{
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z > Area4_MAX_Point_Z[j]) {
				Area4_MAX_Point_Z[j] = _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				Area4_Index[j] = i + j * _ONE_FRAME_POINT_COUNT_;
			}
		}
		for (size_t i = _5TH_AERA_; i < _5TH_AERA_ + _ONCE_STEP_; i++)
		{
			if (_point[i + j * _ONE_FRAME_POINT_COUNT_].z > Area5_MAX_Point_Z[j]) {
				Area5_MAX_Point_Z[j] = _point[i + j * _ONE_FRAME_POINT_COUNT_].z;
				Area5_Index[j] = i + j * _ONE_FRAME_POINT_COUNT_;
			}
		}
	}

	// 对所有极大值点进行计算：找出他们中的最大值点及坐标和最小值点
	UINT Area_Value_Max[5] = { 0 };
	UINT Area_Value_Min[5] = { 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30 };
	UINT Index_Value_Max[5] = { 0 };
	for (size_t i = GiveUpPoints; i < _ROW_NUM_; i++)
	{
		// AREA - 1
		if (Area1_MAX_Point_Z[i] != 0 && Area1_MAX_Point_Z[i] > Area_Value_Max[0]) {
			Area_Value_Max[0] = Area1_MAX_Point_Z[i];
			Index_Value_Max[0] = Area1_Index[i];		//记录下标
		}
		if (Area1_MAX_Point_Z[i] != 0 && Area1_MAX_Point_Z[i] < Area_Value_Min[0])
			Area_Value_Min[0] = Area1_MAX_Point_Z[i];

		// AREA - 2
		if (Area2_MAX_Point_Z[i] != 0 && Area2_MAX_Point_Z[i] > Area_Value_Max[1]) {
			Area_Value_Max[1] = Area2_MAX_Point_Z[i];
			Index_Value_Max[1] = Area2_Index[i];		//记录下标
		}
		if (Area2_MAX_Point_Z[i] != 0 && Area2_MAX_Point_Z[i] < Area_Value_Min[1])
			Area_Value_Min[1] = Area2_MAX_Point_Z[i];

		// AREA - 3
		if (Area3_MAX_Point_Z[i] != 0 && Area3_MAX_Point_Z[i] > Area_Value_Max[2]) {
			Area_Value_Max[2] = Area3_MAX_Point_Z[i];
			Index_Value_Max[2] = Area3_Index[i];		//记录下标
		}
		if (Area3_MAX_Point_Z[i] != 0 && Area3_MAX_Point_Z[i] < Area_Value_Min[2])
			Area_Value_Min[2] = Area3_MAX_Point_Z[i];

		// AREA - 4
		if (Area4_MAX_Point_Z[i] != 0 && Area4_MAX_Point_Z[i] > Area_Value_Max[3]) {
			Area_Value_Max[3] = Area4_MAX_Point_Z[i];
			Index_Value_Max[3] = Area4_Index[i];		//记录下标
		}
		if (Area4_MAX_Point_Z[i] != 0 && Area4_MAX_Point_Z[i] < Area_Value_Min[3])
			Area_Value_Min[3] = Area4_MAX_Point_Z[i];

		// AREA - 5
		if (Area5_MAX_Point_Z[i] != 0 && Area5_MAX_Point_Z[i] > Area_Value_Max[4]) {
			Area_Value_Max[4] = Area5_MAX_Point_Z[i];
			Index_Value_Max[4] = Area5_Index[i];		//记录下标
		}
		if (Area5_MAX_Point_Z[i] != 0 && Area5_MAX_Point_Z[i] < Area_Value_Min[4])
			Area_Value_Min[4] = Area5_MAX_Point_Z[i];
	}

	// 对五个区域分别 做最大值-最小值 以获得最大跳动值
	UINT maxDrump[5] = { 0 };
	UINT maxDrump_X[5] = { 0 };
	UINT maxDrump_Y[5] = { 0 };
	UINT maxDrump_Z[5] = { 0 };
	for (size_t i = 0; i < 5; i++)
	{
		maxDrump[i] = Area_Value_Max[i] - Area_Value_Min[i];
		maxDrump_X[i] = _point[Index_Value_Max[i]].x;
		maxDrump_Y[i] = _point[Index_Value_Max[i]].y;
		maxDrump_Z[i] = _point[Index_Value_Max[i]].z;
	}

	// 寻找出整个检测区域的最大跳动公差，并输出最大跳动点的三维坐标
	UINT index = 0;
	for (size_t i = 0; i < 5; i++)
	{
		if (maxDrump[i] > m_pt_MaxDrump_Value)
		{
			m_pt_MaxDrump_Value = maxDrump[i];
			index = i;
		}
	}
	m_pt_MaxDrump_X = maxDrump_X[index];
	m_pt_MaxDrump_Y = maxDrump_Y[index];
	m_pt_MaxDrump_Z = maxDrump_Z[index];

	// 计算缺帧率（滤波前Z值为0的数目 / 监测区域总点数）
	for (size_t i = 0; i < _ROW_NUM_; i++)
	{
		m_pct_LostFrame += _wholeFrameInfo->lostPoint[i];
	}
	m_pct_LostFrame = m_pct_LostFrame / (_ONE_FRAME_POINT_COUNT_ * _ROW_NUM_);
	return TRUE;
}



// FUC: 滤波： 
//		输入点 求点到直线距离
//		距离 > 阈值 返回FALSE - 意为不合格点 
//		距离 < 阈值 返回TRUE  - 合格点
BOOL CDrillDetect::WaveFilter(int pointX, int pointY, float a, float b, BOOL lostFrame)
{
	// 如果缺帧超过阈值则不做滤波
	if (lostFrame)
	{
		return TRUE;
	}
	// 点到直线距离
	float P2Ldistance = fabs((a * pointX + (-1) * pointY + b) / sqrt(a * a + 1));
	if (P2Ldistance < 900)
	{
		return TRUE;
	}
	return FALSE;
}

// FUC: 存图
//		将点云存储到指定文件夹
BOOL CDrillDetect::AutoSavePly(_Point_Calc_ *_point)
{
	// 将滤波后结果写入ply
	FILE* pFileOut;
	char* timeCur = new char[64];
	char* plyName = new char[256];
	memset(timeCur, 0, sizeof(char) * 64);
	memset(plyName, 0, sizeof(char) * 256);
	timeCur = GetTimeForCur();
	strcat(plyName, "D:\\LogForStraightingMachineDetectionSystem\\");
	strcat(plyName, timeCur);
	strcat(plyName, "_after.ply");
	pFileOut = fopen(plyName, "wb+");
	if (NULL == pFileOut) {
		return FALSE;
	}

	char DstBuf[1024] = { 0 };
	unsigned int nDstBufLen = 0;

	sprintf_s(DstBuf, 1024, "ply\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "format ascii 1.0\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "comment author:SIENECT\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "element vertex %d\n", _ROW_NUM_ * _ONE_FRAME_POINT_COUNT_);
	sprintf_s(DstBuf + strlen(DstBuf), 64, "property float x\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "property float y\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "property float z\n");
	sprintf_s(DstBuf + strlen(DstBuf), 64, "end_header\n");
	nDstBufLen = strlen(DstBuf);
	fwrite(DstBuf, nDstBufLen, 1, pFileOut);

	for (size_t i = 0; i < _ROW_NUM_ * _ONE_FRAME_POINT_COUNT_; i++)
	{
		sprintf(DstBuf, "%.2f %.2f %.2f\n\0", _point[i].x * 10, _point[i].y, _point[i].z);
		nDstBufLen = strlen(DstBuf);
		fwrite(DstBuf, nDstBufLen, 1, pFileOut);
	}
	fclose(pFileOut);

	// 释放空间
	delete[] timeCur;
	delete[] plyName;
	return TRUE;
}

// FUC: 释放空间
BOOL CDrillDetect::FreeMemory(_Point_Calc_ * _point, _Whole_Frame_Info_ * _wholeFrameInfo, _Neighbor_Calc_ * _neighbor_Calc, _FITE_PARA_ * _filePara)
{
	delete[] _point;
	/// whole_Frame_Info_
	for (size_t i = 0; i < 4; i++)
	{
		delete[] _wholeFrameInfo->pt_Avg[i];
		delete[] _wholeFrameInfo->realPoint[i];
	}
	delete[] _wholeFrameInfo->lostPoint;
	delete[] _wholeFrameInfo->pt_realPoint;
	delete[] _wholeFrameInfo->realStartIndex;
	delete[] _wholeFrameInfo->realEndIndex;
	delete _wholeFrameInfo;
	/// neighbor_Calc_
	for (size_t i = 0; i < 4; i++)
	{
		delete[] _neighbor_Calc->pt_Avg_less[i];
		delete[] _neighbor_Calc->realLessPoint[i];
	}
	delete _neighbor_Calc;
	/// file_para
	for (size_t i = 0; i < 3; i++)
	{
		delete[] _filePara->a[i];
		delete[] _filePara->b[i];
	}
	for (size_t i = 0; i < 4; i++)
	{
		delete[] _filePara->point_X[i];
		delete[] _filePara->point_Z[i];
	}
	delete[] _filePara->distance;
	delete _filePara;

	return TRUE;
}

// FUC: 获取当前时间
char* CDrillDetect::GetTimeForCur()
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char * returnTime = (char*)malloc(64 * sizeof(char));
	char * temp = (char*)malloc(32 * sizeof(char));

	memset(returnTime, 0, sizeof(returnTime));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wYear, temp, 10));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wMonth, temp, 10));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wDay, temp, 10));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wHour, temp, 10));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wMinute, temp, 10));
	memset(temp, 0, sizeof(temp));
	strcat(returnTime, _itoa(sys.wSecond, temp, 10));
	memset(temp, 0, sizeof(temp));

	return returnTime;
}

// 算法接口：float* pt_Input_X/pt_Input_Y/pt_Input_Z 输入数据三个点
//			 float threshold_Dis：距离阈值
//			 float threshold_Frame：单帧缺点阈值
//			 BOOL autoSave是否保存点云
BOOL CDrillDetect::Detect(_ORG_POINT_* _Org_point, float threshold_Dis, float threshold_Frame, BOOL autoSave)
{
	// 初始化结构体指针 
	/// 申请空间大小： 1195(帧数) * 700(每帧点数) 
	_Point_Calc_ *_point = new _Point_Calc_[_ROW_NUM_ * _ONE_FRAME_POINT_COUNT_];
	memset(_point, 0, sizeof(_Point_Calc_) * _ROW_NUM_ * _ONE_FRAME_POINT_COUNT_);

	// 第一次遍历 数据剪切
	_Whole_Frame_Info_ * whole_Frame_Info_ = new _Whole_Frame_Info_;
	if (!FirstTraversal(_Org_point->_intputPt_x, _Org_point->_intputPt_y, _Org_point->_intputPt_z, _point, whole_Frame_Info_)){
		return FALSE;
	}

	// 寻找邻域点
	_Neighbor_Calc_ * neighbor_Calc_ = new _Neighbor_Calc_;
	if (!PtNeighborCalc(_point, whole_Frame_Info_, neighbor_Calc_)){
		return FALSE;
	}

	// 第二次遍历：滤波
	_FITE_PARA_ * file_para = new _FITE_PARA_;
	if (!SecondTravFite(_point, whole_Frame_Info_, neighbor_Calc_, file_para, threshold_Dis, threshold_Frame)) {
		return FALSE;
	}

	// 计算 
	UINT GiveUpPoints = 15;
	if (!CalcResult(_point, whole_Frame_Info_, GiveUpPoints)) {
		return FALSE;
	}
	
	// 存图
	if (autoSave){
		AutoSavePly(_point);
	}

	// 资源释放
	if (!FreeMemory(_point, whole_Frame_Info_, neighbor_Calc_, file_para))
	{
		return FALSE;
	}
	

	return TRUE;
}