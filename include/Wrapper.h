/*
 * Wrapper.h
 *
 *  Created on: 29th Jan 2017
 *      Author: Samuel Dudley
 */
#ifndef WRAPPER_H
#define WRAPPER_H


#include<iostream>
#include<string>
#include<sstream>
#include<algorithm>
#include<fstream>
#include<vector>
#include<chrono>
#include<iomanip>
#include<opencv2/core.hpp>
#include<cConverter.h>

#include<cSystem.h>
#include<misc.h>
#include<boost/python.hpp>

// numpy
//#include "numpy/ndarrayobject.h"
class CSystemWrapper: public MultiColSLAM::cSystem {
public:
		MultiColSLAM::cTracking* getTracker();
};

class Wrapper
{
	public:
		Wrapper(std::string strVocFile, std::string strConfigFile,  std::string path2MCScalibrationFiles); // wrapper constructor
		//void initialize(const bool bUseViewer = true, const bool reuse= false, const string & mapFilePath = "");
		void initialize(const bool bUseViewer = true);
		void shutdown();

		cv::Matx<double, 6, 1> hom2cayley(cv::Matx<double, 4, 4> M)
		{
			// convert 4x4
				return MultiColSLAM::hom2cayley<double>(M);
		};


		//TODO: make it private?
		void configure(std::string strConfigFile);
		//void track(const std::vector<cv::Mat>& imgSet, const double &timestamp);
		void track(const boost::python::list& imgSet, const double &timestamp);
		int  getStatus();
		void reset();
		void getCurrentFrame();
		bool getIsInitialized();
		void saveTrajectory(const string &filename);

	public:
		std::string msg;
		std::string vocabularyFilePath;
		std::string configurationFilePath;
		std::string path2MCScalibrationFiles;

		bool isInitialized;

		//xiAPIplusCameraOcv cam;
		cv::Mat src;
		cv::Mat im;

		cv::Mat currentFrame; //im with tracking visualization drawn on it
		cv::FileStorage fsConfiguration;

		const double tframe = 0.1;


		cv::Mat cameraMatrix;
		cv::Mat distorsionCoeff;


	private:
		bool initialized();

	private:
		MultiColSLAM::cSystem* SLAM;


};

#endif // WRAPPER_H
