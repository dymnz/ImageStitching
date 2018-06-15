/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef IPOINT_H
#define IPOINT_H

#include <vector>
#include <math.h>
#include "opencv2/core/core.hpp"

using namespace cv;
//-------------------------------------------------------

class Ipoint; // Pre-declaration
typedef std::vector<Ipoint> IpVec;
typedef std::vector<std::pair<Ipoint, Ipoint> > IpPairVec;

//-------------------------------------------------------

//! Ipoint operations
void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches);
int translateCorners(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4]);

//-------------------------------------------------------

class Ipoint {

public:

	//! Destructor
	~Ipoint() {};

	//! Constructor
	Ipoint() : orientation(0) {};

	//! Gets the distance in descriptor space between Ipoints
	float operator-(const Ipoint &rhs)
	{
		float sum = 0.f;
		for (int i = 0; i < 64; ++i)
			sum += (this->descriptor[i] - rhs.descriptor[i])*(this->descriptor[i] - rhs.descriptor[i]);
		return sqrt(sum);
	};

	//! Coordinates of the detected interest point
	float x, y;

	//! Detected scale
	float scale;

	//! Orientation measured anti-clockwise from +ve x-axis
	float orientation;

	//! Sign of laplacian for fast matching purposes
	int laplacian;

	//! Vector of descriptor components
	float descriptor[64];

	//! Placeholds for point motion (can be used for frame to frame motion analysis)
	float dx, dy;

	//! Used to store cluster index
	int clusterIndex;

	void transform(Mat h)
	{
		x = h.at<double>(0, 0)*x + h.at<double>(0, 1)*x + h.at<double>(0, 2)*x;
		y = h.at<double>(1, 0)*y + h.at<double>(1, 1)*y + h.at<double>(1, 2)*y;
	}
};

class BaseImage {
public:
	Mat *image;
	Mat *homography;
	Mat *mask;
	void assignImage(Mat *im) { image = im; }
	void assignImage(Mat im) { image = new Mat(im); }

	BaseImage(IplImage *ipImage)
	{
		image = new Mat(ipImage, true);
	}
	~BaseImage()
	{
		delete image;
	}

};
class Matcher{
public:
	Matcher(IpPairVec matches, int iNum1, int iNum2,
		BaseImage *im1, BaseImage *im2)
	{
		this->matches = matches;
		this->iNum1 = iNum1;
		this->iNum2 = iNum2;
		this->image1 = im1;
		this->image2 = im2;
	}
	void clear() { iNum1 = iNum2 = -1; }
	int getI1() { return iNum1; }
	int getI2() { return iNum2; }
	void setI1(int i) { iNum1 = i; }
	void setI2(int i) { iNum2 = i; }
	IpPairVec getMatches(){return matches;}
private:
	BaseImage *image1, *image2;
	int iNum1, iNum2;
	IpPairVec matches;
};

class Route{
public:
	vector< vector<int> > route;
	vector< int > routeWeight;
	vector< int > routeWeightAvg;
};




class MatchTracker {
public:
	vector < Route > routes;
	vector < vector <int> > pairNum;
	vector < vector <IpPairVec> > pairFP;
	vector < vector <Mat> > pairHomography;

	MatchTracker(int size) 
	{
		pairNum.resize(size);
		pairFP.resize(size);
		routes.resize(size);
		pairHomography.resize(size);
		for (int i = 0; i < size; i++)
		{
			pairNum[i].resize(size);
			pairFP[i].resize(size);
			pairHomography[i].resize(size);
			pairFP[i].clear();
			for(int j = 0;j < size;j++)
			{
				pairHomography[i][j] =  Mat::zeros(3, 3, CV_64F);
			}
		}
	}
	void assignFPNum(int i, int r, int fp) { pairNum[i][r] = fp; pairNum[r][i] = fp;  }
	void assignHomography(int i, int r, Mat &H){ pairHomography[i][r] = H; }
	void assignFPPair(int i, int r, IpPairVec fp) { pairFP[i][r] = fp; }
	vector<int>& getPairNum(int i) { return pairNum[i]; }
	int getPairNum(int i, int r){ return pairNum[i][r]; }
	Mat getPairHomography(int i, int r){ return pairHomography[i][r];}
	vector<IpPairVec>& getPairFP(int i) { return pairFP[i]; }
	IpPairVec& getPairFP(int i, int r)
	{
		if (!pairFP[i][r].empty())
			return pairFP[i][r];
		if (!pairFP[r][i].empty())
			return pairFP[r][i];
		return IpPairVec();
	}
	void assignRoute(int image, vector<int > r){ routes[image].route.push_back(r); }
	Route& getRoute(int i){ return routes[i]; }
	int getSize() { return pairNum.size(); }

};



//-------------------------------------------------------


#endif
