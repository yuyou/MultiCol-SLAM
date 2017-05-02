
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <mutex>

#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "cTracking.h"
#include "cConverter.h"
#include "cam_model_omni.h"
#include "cSystem.h"

using namespace std;

void LoadImagesAndTimestamps(
	const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps);

// Camera number
const int nrCams = 3;

int main(int argc, char **argv)
{
	if (argc != 5)
	{
		cerr << endl << "Usage: ./ozo vocabulary_file slam_settings_file path_to_calibrations path_to_img_sequence" << endl;
		return 1;
	}

	string path2voc = string(argv[1]);
	string path2settings = string(argv[2]);
	string path2calibrations = string(argv[3]);
	string path2imgs = string(argv[4]);

	// --------------
	// 1. Tracking settings
	// --------------
	cv::FileStorage frameSettings(path2settings, cv::FileStorage::READ);

	int traj = (int)frameSettings["traj2Eval"];
	string trajs = to_string(traj);
	const int endFrame = (int)frameSettings["traj.EndFrame"];
	const int startFrame = (int)frameSettings["traj.StartFrame"];

	// --------------
	// 4. Load image paths and timestamps
	// --------------
	vector<vector<string>> imgFilenames;
	vector<double> timestamps;
	LoadImagesAndTimestamps(startFrame, endFrame, path2imgs, imgFilenames, timestamps);

	int nImages = imgFilenames[0].size();

	MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	//const int nrCams = static_cast<int>(imgFilenames.size());
	cout << "Total cameras: " << nrCams << endl;

	//std::vector<cv::Mat> imgs(nrCams);
	std::vector<cv::Mat> imgs(8);
	for (int ni = 0; ni < nImages; ni++)
	{
			/*
	    if (ni == 30){
	        // pause
	        cout << "Pause 60 seconds " << nrCams << endl;
	        sleep(60); // format is sleep(x); where x is # of seconds.
	    }
			*/
		// Read image from file
		//std::vector<bool> loaded(nrCams);
		for (int c = 0; c < nrCams; ++c)
		{
			cout << endl << "Load image " << imgFilenames[c][ni] << endl;
			imgs[c] = cv::imread(imgFilenames[c][ni], CV_LOAD_IMAGE_GRAYSCALE);
			//imgs[c] = cv::imread(imgFilenames[c][ni], CV_LOAD_IMAGE_COLOR);
			if (imgs[c].empty())
			{
				cerr << endl << "Failed to load image at: " << imgFilenames[c][ni] << endl;
				// Stop all threads
	      MultiSLAM.Shutdown();
				return -1;
			}
		}
		double tframe = timestamps[ni];
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// Pass the image to the SLAM system

		//cout << "Pass the images to the SLAM system" << endl;

		MultiSLAM.TrackMultiColSLAM(imgs, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;

		// Wait to load the next frame
		double T = 0;
		if (ni < nImages - 1)
			T = timestamps[ni + 1] - tframe;
		else if (ni > 0)
			T = tframe - timestamps[ni - 1];
		//std::this_thread::sleep_for(std::chrono::milliseconds(30));

		if (ttrack < T)
			std::this_thread::sleep_for(std::chrono::milliseconds(
			static_cast<long>((T - ttrack))));
	}

	// Stop all threads
	MultiSLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	MultiSLAM.SaveMKFTrajectoryLAFIDA("MKFTrajectory.txt");

	return 0;
}


void LoadImagesAndTimestamps(const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps)
{
	vstrImageFilenames.resize(8);
	ifstream fTimes;
	string strPathTimeFile = path2imgs + "/images_and_timestamps.txt";
	cout << "Reading frame list:" << strPathTimeFile << endl << endl;
	fTimes.open(strPathTimeFile.c_str());
	string line;

	int cnt = 1;
	while (std::getline(fTimes, line))
	{
		if (cnt >= startFrame && cnt < endFrame) // skip until startframe
		{
			std::istringstream iss(line);
			double timestamp;
			string pathimg1, pathimg2, pathimg3, pathimg4, pathimg5, pathimg6, pathimg7, pathimg8;
			//if (!(iss >> timestamp >> pathimg1 >> pathimg2 >> pathimg3 >> pathimg4))
			if (!(iss >> timestamp >> pathimg1 >> pathimg2 >> pathimg3 >> pathimg4 >> pathimg5 >> pathimg6 >> pathimg7 >> pathimg8))
				break;

			vTimestamps.push_back(timestamp);
			vstrImageFilenames[0].push_back(path2imgs + '/' + pathimg1);
			vstrImageFilenames[2].push_back(path2imgs + '/' + pathimg2);
			vstrImageFilenames[1].push_back(path2imgs + '/' + pathimg3);
			vstrImageFilenames[3].push_back(path2imgs + '/' + pathimg4);
			vstrImageFilenames[4].push_back(path2imgs + '/' + pathimg5);
			vstrImageFilenames[5].push_back(path2imgs + '/' + pathimg6);
			vstrImageFilenames[6].push_back(path2imgs + '/' + pathimg7);
			vstrImageFilenames[7].push_back(path2imgs + '/' + pathimg8);
		}
		++cnt;

	}
}
