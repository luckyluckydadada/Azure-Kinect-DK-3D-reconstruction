#include "base.h"

int main(int argc, char** argv)
{
    CAMERA_INTRINSIC_PARAMETERS cameraParams{ 913.451, 913.122, 953.516, 554.09, 1 };  //相机内参
    int frameNum = 3;
    PointCloud::Ptr fusedCloud(new PointCloud());
    for (int idx = 0; idx < frameNum; idx++)
    {
        string rgbPath = "D:\\data\\rgb\\" + to_string(idx) + ".png";   //图像以数字命名
        string depthPath = "D:\\data\\depth\\" + to_string(idx) + ".png";
        FRAME frm;
        frm.rgb = cv::imread(rgbPath);
        if (frm.rgb.empty()) {
            cerr << "Fail to load rgb image!" << endl;
        }
        frm.depth = cv::imread(depthPath, -1);
        if (frm.depth.empty()) {
            cerr << "Fail to load depth image!" << endl;
        }

        if (idx == 0)
        {
            fusedCloud = image2PointCloud(frm.rgb, frm.depth, cameraParams);
        }
        else
        {
            fusedCloud = pointCloudFusion(fusedCloud, frm, cameraParams);
        }
    }
    pcl::io::savePCDFile("D:\\data\\fusedCloud.pcd", *fusedCloud);
    pcl::io::savePLYFile("D:\\data\\fusedCloud.ply", *fusedCloud);
    cout << "END!" << endl;
    return 0;
}
