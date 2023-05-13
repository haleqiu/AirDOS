#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void LoadSegs(const string &strPathToSequence, vector<string> &vstrSegLeft,
                vector<string> &vstrSegRight, const int nTimes);
void LoadDepths(const string &strPathToSequence, vector<string> &vstrImageDepth, const int nTimes);

int main(int argc, char **argv)
{
    if(argc != 6 && argc != 5)
    {
      cerr << endl << "Usage: ./stereo_kitti_human path_to_vocabulary path_to_settings path_to_sequence path_to_save_trajectory [path_to_save_map?]" << endl;
      return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> vstrSegLeft;
    vector<string> vstrSegRight;
    vector<string> vstrImageDepth;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    LoadSegs(string(argv[3]), vstrSegLeft, vstrSegRight, vTimestamps.size());
  LoadDepths(string(argv[3]), vstrImageDepth, vTimestamps.size());

    int nImages = vstrImageLeft.size();
    int nStartImage = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // Here, set the flag OffLine
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    int iIsOffline = fsSettings["System.IsOffline"];
    bool bIsOffline = iIsOffline;

    if (bIsOffline)
        std::cout << "the system is running offline without multi-threading!!!!!!!!!!!!!!!"<<std::endl;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, bIsOffline);
    SLAM.folder_path = string(argv[3]);

    // Mannualy set the end point of the sequence, 
    int iClip = fsSettings["Schedular.nEndImage"];
    bool isClip = iClip;
    if (isClip){
      std::cerr << iClip << '\n';
      if (iClip < nImages){
        nImages = iClip;
      }
      nStartImage = fsSettings["Schedular.nStartImage"];
    }

    // The flag to determine if using the semantic segmentation
    int iMask = fsSettings["System.IsMask"];
    bool bMask = iMask;

    SLAM.ReadAllHumanPoses(nImages);
    SLAM.ReadTrackId(nImages);
    // Vector for tracking time statistics


    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, imLeftSeg, imRightSeg, imDepth;

    for(int ni=nStartImage; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);
        if (bMask)
        {
        cout << "Enable the Mask RCNN module ..." << endl;
        imLeftSeg = cv::imread(vstrSegLeft[ni],cv::IMREAD_GRAYSCALE);
        imRightSeg = cv::imread(vstrSegRight[ni],cv::IMREAD_GRAYSCALE);
        }
        
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereoHuman(imLeft,imRight,imLeftSeg,imRightSeg,imDepth,tframe,ni);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        if (!bIsOffline){
            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];
            std::cerr << "time used"<<ttrack << '\n';
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }

    if (argc == 5) SLAM.BeforeEnd();
    else SLAM.BeforeEnd(string(argv[5]));

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(argv[4]);
    return 0;
}

void LoadDepths(const string &strPathToSequence, vector<string> &vstrImageDepth, const int nTimes)
{
    string strPrefix = strPathToSequence + "/depth/";

    vstrImageDepth.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i+8;
        vstrImageDepth[i] = strPrefix + ss.str() + ".png";
    }
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadSegs(const string &strPathToSequence, vector<string> &vstrSegLeft,
                vector<string> &vstrSegRight, const int nTimes)
{
    string strPrefixLeft = strPathToSequence + "/rcnnseg_image_0/";
    string strPrefixRight = strPathToSequence + "/rcnnseg_image_1/";

    vstrSegLeft.resize(nTimes);
    vstrSegRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrSegLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrSegRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
