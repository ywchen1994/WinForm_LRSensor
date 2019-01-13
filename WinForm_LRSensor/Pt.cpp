#include "Pt.h"

Pt::Pt()
{
	x = 0;
	y = 0;
	range = 0;
	theta = 0;
	velcity = 0;
	is_KF_init = false;
	is_inRadarRange = false;
	// rng.fill(KF.statePost,cv::RNG::UNIFORM, 0, false);

}
Pt::Pt(double _x, double _y)
{
	x = _x;
	y = _y;
	range = 0;
	theta = 0;
	is_visited = false;
	isCore = false;
	clusterId = -1;
	is_KF_init = false;
	is_inRadarRange = false;
}

Pt::Pt(double _x, double _y, double _range, double _theta)
{
	x = _x;
	y = _y;
	range = _range;
	theta = _theta;
	is_visited = false;
	isCore = false;
	clusterId = -1;
	is_KF_init = false;
}
void Pt::KF_initial()
{

	KF = cv::KalmanFilter(4, 2, 0, CV_64FC1);
	KF.transitionMatrix = (cv::Mat_<double>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	setIdentity(KF.measurementMatrix);                                            //測量矩陣H                 
	setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));                       //系統雜訊矩陣Q
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));                   //量測雜訊方差R
	setIdentity(KF.errorCovPost, cv::Scalar::all(1));                             //後驗錯誤估計斜方矩陣P
	KF.statePost = (cv::Mat_<double>(4, 1) << x, y, 0, 0);
	is_KF_init = true;
}
inline double get_Distance(Pt P1, Pt P2)
{
	return sqrt(pow((P1.x - P2.x), 2) + pow((P1.y - P2.y), 2));
}
void CorePointCluster(Pt &P, vector<Pt>&RawData, uint16_t clusterId)
{

	if (!P.isCore) {
		P.clusterId = clusterId;
		return;
	}
	for (size_t i = 0; i < P.ArrivalPoints.size(); i++)
	{
		Pt &dstp = RawData[P.ArrivalPoints[i]];
		if (!dstp.is_visited)
		{
			dstp.is_visited = true;
			dstp.clusterId = clusterId;
			if (dstp.isCore)
				CorePointCluster(dstp, RawData, clusterId);
		}
	}
}
int DBSCAN(std::vector<Pt>& _vec, double Eps, int MinPts)
{
	uint32_t dataLength = _vec.size();


	for (uint32_t i = 0; i < dataLength; i++)
	{


		for (uint32_t j = 0; j < dataLength; j++)
		{
			if (i != j)
			{
				if (get_Distance(_vec[i], _vec[j]) <= Eps)
					_vec[i].ArrivalPoints.push_back(j);
			}
			else
			{
				continue;
			}
		}
		if (_vec[i].ArrivalPoints.size() >= MinPts)
			_vec[i].isCore = true;
		else
			_vec[i].isCore = false;
	}

	unsigned int SetClusterId = 0;
	for (unsigned int i = 0; i < _vec.size(); i++)
	{

		if (!_vec[i].is_visited && _vec[i].isCore)
		{
			_vec[i].is_visited = true;
			_vec[i].clusterId = SetClusterId;
			CorePointCluster(_vec[i], _vec, SetClusterId);
			SetClusterId++;
		}
		else
			_vec[i].is_visited = true;
	}
	return SetClusterId;
}
bool is_Cluster(Pt P1, Pt P2, double eps)
{
	return(get_Distance(P1, P2) < eps) || (abs(P1.y - P2.y) < 200 && abs(P1.x - P2.x) < 200);
}
vector<vector<Pt>>Cluster2List(vector<Pt>&XYcord, int NoObj)
{
	vector<vector<Pt> >ListTemp;
	ListTemp.resize(NoObj);
	for (uint i = 0; i < XYcord.size(); i++)
	{
		if (XYcord[i].clusterId != -1)
			ListTemp[XYcord[i].clusterId].push_back(XYcord[i]);
	}
	return 	ListTemp;
}
bool predicate(Pt P1, Pt P2, double eps)
{
	return  get_Distance(P1, P2) < eps;
}
bool predicate(Pt P1, Pt P2)
{
	return   (abs(P1.y - P2.y)<200) && (abs(P1.x - P2.x) < 100) && (abs(P1.theta - P2.theta) < 3);
}
int EuclidCluster(vector<Pt>& _vec, double eps)
{
	int i, j, N = _vec.size();
	const Pt* vec = &_vec[0];

	const int PARENT = 0;
	const int RANK = 1;

	vector<int> _nodes(N * 2);
	int(*nodes)[2] = (int(*)[2])&_nodes[0];

	for (i = 0; i < N; i++)
	{
		nodes[i][PARENT] = -1;
		nodes[i][RANK] = 0;
	}
	for (i = 0; i < N; i++)
	{
		_vec[i].isCore = true;
		int root = i;

		// find root
		while (nodes[root][PARENT] >= 0)
			root = nodes[root][PARENT];

		for (j = 0; j < N; j++)
		{
			if (i == j || !predicate(vec[i], vec[j]))
				continue;
			int root2 = j;

			while (nodes[root2][PARENT] >= 0)
				root2 = nodes[root2][PARENT];

			if (root2 != root)
			{
				// unite both trees
				int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];
				if (rank > rank2)
					nodes[root2][PARENT] = root;
				else
				{
					nodes[root][PARENT] = root2;
					nodes[root2][RANK] += rank == rank2;
					root = root2;
				}
				//assert(nodes[root][PARENT] < 0);

				int k = j, parent;

				// compress the path from node2 to root
				while ((parent = nodes[k][PARENT]) >= 0)
				{
					nodes[k][PARENT] = root;
					k = parent;
				}

				// compress the path from node to root
				k = i;
				while ((parent = nodes[k][PARENT]) >= 0)
				{
					nodes[k][PARENT] = root;
					k = parent;
				}
			}
		}
	}
	for (unsigned int i = 0; i < N; i++)
		_vec[i].clusterId = 0;
	//labels.push_back(0);
	int nclasses = 0;

	for (i = 0; i < N; i++)
	{
		int root = i;
		while (nodes[root][PARENT] >= 0)
			root = nodes[root][PARENT];
		if (nodes[root][RANK] >= 0)
			nodes[root][RANK] = ~nclasses++;
		_vec[i].clusterId = ~nodes[root][RANK];
	}
	return nclasses;
}
vector<vector<Pt>>Cluster2List_ContinuousAngle(vector<Pt>&XYcord)
{
	vector<vector<Pt> >ListTemp;
	vector<Pt> ContinuousPt;
	for (uint i = 0; i < XYcord.size() - 2; i++)
	{
		if (predicate(XYcord[i], XYcord[i + 1]) || predicate(XYcord[i], XYcord[i + 2]))
		{
			XYcord[i].isCore = true;
			ContinuousPt.push_back(XYcord[i]);
		}
		else
		{
			if (ContinuousPt.size() > 1)
				ListTemp.push_back(ContinuousPt);
			ContinuousPt.resize(0);
		}
	}

	return ListTemp;
}
void FindRectangle(vector<Pt> clusterPt,double &W,double &H,double &angle,Pt &refPt)
{

	valarray<double> x,y;
	x.resize(clusterPt.size());
	y.resize(clusterPt.size());
	double minDistant=800000;
	for (double i = 0; i < 91; i++)
	{
		
		for (uint j = 0; j < clusterPt.size(); j++)
		{
			Pt rotate = CoordinateRotation(i, clusterPt[j]);
			x[j] = rotate.x;
			y[j] = rotate.y;
		}
		if (minDistant > (x.max() - x.min()))
		{
			minDistant = (x.max() - x.min());
			W = minDistant;
			H = y.max() - y.min();
			angle = i;
		}
	}

}
Pt CoordinateRotation(double degree, Pt P)
{
	Pt Ans;
	Ans.x = cos(degree*M_PI / 180)*P.x -sin(degree*M_PI / 180)*P.y;
	Ans.y = sin(degree*M_PI / 180)*P.x + cos(degree*M_PI / 180)*P.y;
	return Ans;
}