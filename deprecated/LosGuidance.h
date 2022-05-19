// LosGuidance.cpp : DLL 응용 프로그램을 위해 내보낸 함수를 정의합니다.
//

#include <opencv2/opencv.hpp>

inline double GetDistanceMeter(cv::Point2d pt1, cv::Point2d pt2)
{
	return sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0));
}

inline int LOSGuidance(cv::Point2d _fPtArr[], cv::Point2d& _fPtCurr, double _fRadius, cv::Point2d& _fPtLos)
{
	// Line of Sight 함수
	// input: initial point
	//		    target point
	// output: LoS angle

	double x_k_1 = _fPtArr[0].x;
	double y_k_1 = _fPtArr[0].y;
	double x_k = _fPtArr[1].x;// + (int)( path_length*sin(DEG2RAD*theta) );
	double y_k = _fPtArr[1].y;// + (int)( path_length*cos(DEG2RAD*theta) );

	double robot_radius = _fRadius; // [m]
	double robot_coa = _fRadius;
	double psi_los = 0;
	double path_distance, wpdistance;
	double delta_x;
	double delta_y;
	double x_los, y_los;
	double e, f, g, a, b, c, d;

	double x = _fPtCurr.x;
	double y = _fPtCurr.y;

	//check distance to current way-point
	path_distance = (sqrt(((x_k - x_k_1)*(x_k - x_k_1) + (y_k - y_k_1)*(y_k - y_k_1))));
	wpdistance = (sqrt(((x_k - x)*(x_k - x) + (y_k - y)*(y_k - y))));

	if (wpdistance <= robot_coa)
	{
		return 2;
	}

	//psi_los calculation    
	delta_x = x_k - x_k_1;
	delta_y = y_k - y_k_1;

	if (delta_x == 0)
	{
		if (abs(x - x_k_1) > robot_radius)
		{
			x_los = x_k;
			y_los = y_k;
			return 1;
		}

		x_los = x_k_1;  //x_los = x_k;

		if (delta_y > 0)
		{
			y_los = y + sqrt(robot_radius*robot_radius - (x - x_los)*(x - x_los));
		}
		else
		{ //delta_y <= 0
			y_los = y - sqrt(robot_radius*robot_radius - (x - x_los)*(x - x_los));
		}

	}
	else
	{  //delta_x ~= 0
		d = delta_y / delta_x;
		e = x_k_1;
		f = y_k_1;
		g = -d*e + f;

		a = 1 + d*d;
		b = 2 * (d*g - d*y - x);
		c = x*x + y*y + g*g - (robot_radius)*(robot_radius)-2 * g*y;


		if ((b*b - 4 * a*c) > 0)
		{
			if (delta_x > 0)
			{
				x_los = ((-b + sqrt(b*b - 4 * a*c))) / (2 * a);
			}
			else
			{ //delta_x < 0
				x_los = ((-b - sqrt(b*b - 4 * a*c))) / (2 * a);
			}
			y_los = (double)d*(x_los - e) + f;
			int testline123 = 2;
		}
		else	//x_los,y_los가 범위안에 안들었을때의 예외 처리 다음 via point 
		{
			//TRACE("Using Losguide with Goal Point\n");			
			x_los = x_k;
			y_los = y_k;
			return 1;
		}
	}

	//psi_los = atan2(y_los - y, x_los - x) * 180.0 / CV_PI ;		// in degrees
	_fPtLos.x = (double)x_los;
	_fPtLos.y = (double)y_los;

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Return: 0 -> Success, pt_los에 결과 저장
//         -1 -> Fail
inline int WptCtrl(cv::Point2d& pt_curr, cv::Point2d& pt_target, cv::Point2d& pt_los, float& f_los_radius)
{
	static bool bInit = true;
	static bool bValidTarget = false;
	static cv::Point2d pt_wpts[2];
	static cv::Point2d pt_target_saved = cv::Point2d(0.0, 0.0);
	static float m_fLosRadius = 3.0f;						// [m] LOS guidance radius
	
	// target point 변경시 변수 초기화
	if (GetDistanceMeter(pt_target, pt_target_saved) > 0.01f) bInit = true;

	if(bInit)
	{
		pt_wpts[0] = pt_curr;
		pt_wpts[1] = pt_target;
		pt_target_saved = pt_target;
		
		bInit = false;
		bValidTarget = true;
	}
	
	if (bValidTarget)
	{
		// 원 안에 waypoint가 들어오면 다음 waypoint 추종
		if (GetDistanceMeter(pt_wpts[1], pt_curr) < f_los_radius)
		{
			bValidTarget = false;
			return -1;
		}

		if (LOSGuidance(pt_wpts, pt_curr, f_los_radius, pt_los) == 1)
		{
			pt_wpts[0] = pt_curr;
			LOSGuidance(pt_wpts, pt_curr, f_los_radius, pt_los);
		}
	}
	else
	{
		bValidTarget = false;
		return -1;	// fail to calculate los position
	}
	
	return 0;
}
