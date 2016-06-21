#pragma once

#include "stdafx.h"

// Need to Download OpenCV and locate 
#include <C:/opencv/build/include/opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "define.h"

using namespace cv;
using namespace std;


class CImage {

	private :
		

	public :
		CImage();
		~CImage();
		int main(int argc, char **argv);
		float cv_distance(Point2f P, Point2f Q);					// Get Distance between two points
		float cv_lineEquation(Point2f L, Point2f M, Point2f J);		// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
		float cv_lineSlope(Point2f L, Point2f M, int& alignement);	// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
		void cv_getVertices(vector<vector<Point> > contours, int c_id,float slope, vector<Point2f>& X);
		void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
		void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
		bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
		float cross(Point2f v1,Point2f v2);
};