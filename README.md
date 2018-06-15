### Image stiticing based on OpenSURF and seam-line stitching

*Original work done in Visual Studio w/ OpenCV on Windows.*

#### Structure

* OpenSURF
  * fasthessian
  * integral
  * ipoint
  * kmeans
  * surf
  * utils

* Entry point
  * main
  	* `int mainStaticStitching(int imageCount, char *imageStr[])`

 * Seam-line stitching
   * utils
     * `void blendImage(int type,Mat &image1, Mat &image2, Mat &imageResult, Mat &t_mask1, Mat &t_mask2, int dX, int dY)`
       * Entry point for the following
	     * `Mat verticalBlending(cv::Mat image1, cv::Mat image2, Mat mask1, Mat mask2)`
	     * `Mat horizontalBlending(cv::Mat image1, cv::Mat image2, Mat mask1, Mat mask2, int dY)`





---

### Seam-line search in `Mat verticalBlending(cv::Mat image1, cv::Mat image2, Mat mask1, Mat mask2)`

Find the minimal error seam-line between pt1 and pt2

```
	enum direction{TOPLEFT, TOP, TOPRIGHT, LEFT, CURRENT, RIGHT, BOTTOMLEFT, BOTTOM, YO};

	direction **dirMap;
	dirMap = new direction * [image1.rows];
	for(int i = 0;i < image1.rows;i++)
	{
		dirMap[i] = new direction [image1.cols];
		for(int j = 0;j < image1.cols;j++)
		{
			dirMap[i][j] = YO;
		}
	}

	for (int i = pt1.y; i <= pt2.y; i++)
	{
		if( i == pt1.y )
		{
			for (int j = 0; j < errorMap.cols; j++)
			{
				if (andMasks.at<unsigned char>(i, j)==0)
					continue;
				if(j == pt1.x)
				{
					errorMap.at<double>(i, j) = ComputeError(image1, image2, i, j);
					dirMap[i][j] = CURRENT;
				}else
				{
					ComputeError(i, j, image1, image2, dirMap, errorMap);
				}
			}
			for(int j = errorMap.cols-1 ; j>=0 ; j--)
			{
				if (andMasks.at<unsigned char>(i, j)==0)
					continue;
				if(j == pt1.x)
				{
					errorMap.at<double>(i, j) = ComputeError(image1, image2, i, j);
					dirMap[i][j] = CURRENT;
				}else
				{
					ComputeError(i, j, image1, image2, dirMap, errorMap);
				}
			}
		}
		else
		{
			for(int j = 0 ; j<errorMap.cols ; j++)
			{
				if (andMasks.at<unsigned char>(i, j)==0)
					continue;
				ComputeError(i, j, image1, image2, dirMap, errorMap);
			}
			for(int j = errorMap.cols-1 ; j>=0 ; j--)
			{
				if (andMasks.at<unsigned char>(i, j)==0)
					continue;
				ComputeError(i, j, image1, image2, dirMap, errorMap);
			}
		}
	}
	for (int i = pt1.y; i <= pt2.y; i++)
	{
		for (int j = 0; j < errorMap.cols; j++)
			if(j+1<errorMap.cols && j>0
				&& dirMap[i][j-1]==RIGHT && dirMap[i][j]==LEFT)
			{
				dirMap[i][j-1] = LEFT;
				dirMap[i][j] = RIGHT;
			}
	}


	void ComputeError(int i, int j, Mat &image1, Mat &image2, direction **dirMap, Mat &errorMap)
	{
		double eCurrent = ComputeError(image1, image2, i, j);
		//TL T TR L R
		double errors[5] = {getDPError(i-1, j-1, errorMap, dirMap),
						getDPError(i-1, j, errorMap, dirMap),
						getDPError(i-1, j+1, errorMap, dirMap),
						getDPError(i, j-1, errorMap, dirMap),
						getDPError(i, j+1, errorMap, dirMap),};
		double minError = 110000;
		int minDir = -1;
		for(int i=0 ; i<5 ; i++)
		{
			if(errors[i]==-1)
				continue;
			if(errors[i]<minError)
			{
				minError = errors[i];
				minDir = i;
			}
		}
		if(minDir==-1)
		{
			dirMap[i][j] = CURRENT;
			errorMap.at<double>(i, j) = eCurrent;
		}
		else
		{
			switch(minDir)
			{
			case 0:
				dirMap[i][j]=TOPLEFT;
				break;
			case 1:
				dirMap[i][j]=TOP;
				break;
			case 2:
				dirMap[i][j]=TOPRIGHT;
				break;
			case 3:
				dirMap[i][j]=LEFT;
				break;
			case 4:
				dirMap[i][j]=RIGHT;
				break;
			default:
				dirMap[i][j]=CURRENT;
			}
			errorMap.at<double>(i, j) = eCurrent + minError;
		}

	}

```