/**
 * @brief fake rgbd (based on rgbd)
 *
 * @input: stereo image
 * @details convert original stereo image to rgbd,
 *      then call the rgbd
 *
 * @demo: kitti dataset 00
 *
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>
// for fake rgbd
#include <opencv2/opencv.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<double> &vTimestamps);

void Stereo2RGBD(const cv::Mat &imLeft, const cv::Mat &imRight,
                 cv::Mat &imRGB, cv::Mat &imD)
{
    // 双目转视差
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // 神奇的参数
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(imLeft, imRight, disparity_sgbm);
    // disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
    disparity_sgbm.convertTo( , CV_32F, 1.0 / 16.0f);

    // // 视差转深度
    // const float bf = 386.1448; // stereo baseline times fx
    // for (int v = 0; v < imLeft.rows; v++)
    //     for (int u = 0; u < imLeft.cols; u++)
    //     {
    //         if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
    //         {
    //             // TODO
    //             // imD.ptr<float>(v)[u] = 1000;
    //             continue;
    //         }
    //         // double depth = fx * b / (disparity.at<float>(v, u));
    //         double depth = bf / (disparity.at<float>(v, u));
    //         imD.ptr<float>(v)[u] = depth;
    //     }
}

int main(int argc, char **argv)
{
    // if (argc != 4)
    // {
    //     cerr << endl
    //          << "Usage: ./stereo_kitti_as_rgbd path_to_vocabulary path_to_settings path_to_sequence" << endl;
    //     return 1;
    // }
    argv[1] = "../Vocabulary/ORBvoc.txt";
    argv[2] = "../Examples/Stereo_as_rgbd/KITTI00-02_as_rgbd.yaml";
    argv[3] = "/mnt/hgfs/code/data_odometry_gray/00/";

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Check consistency in the number of images and depthmaps
    // int nImages = vstrImageFilenamesRGB.size();
    // if (vstrImageFilenamesRGB.empty())
    // {
    //     cerr << endl
    //          << "No images found in provided path." << endl;
    //     return 1;
    // }
    // else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
    // {
    //     cerr << endl
    //          << "Different number of images for rgb and depth." << endl;
    //     return 1;
    // }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imLeft, imRight;               // kitti data
    cv::Mat imRGB, imD(376, 1241, CV_32F); // converter result for LK+ORB

    for (int ni = 0; ni < nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imLeft.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // stereo to rgbd
        Stereo2RGBD(imLeft, imRight, imRGB, imD);

        // Pass the image to the SLAM system
        // SLAM.TrackRGBD(imRGB, imD, tframe);
        SLAM.TrackRGBD(imLeft, imD, tframe);

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

// for kitti data set
void LoadImages(const string &strPathToSequence, // /mnt/hgfs/code/data_odometry_gray/00/
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<double> &vTimestamps)
{
    ifstream fTimes;

    // /mnt/hgfs/code/data_odometry_gray/00/times.txt
    string strPathTimeFile = strPathToSequence + "/times.txt";

    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s); // 读入行
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t); // 相对时间，从0开始，单位秒
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
