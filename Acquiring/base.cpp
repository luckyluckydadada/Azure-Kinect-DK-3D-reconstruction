#include "Base.h"
#include <string>
#include <iostream>

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr cloud(new PointCloud);
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];

            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}


PointCloud::Ptr pointCloudFusion(PointCloud::Ptr& original, FRAME& newFrame, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr newCloud(new PointCloud()), transCloud(new PointCloud());
    newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
    *original += *newCloud;
    return original;
}


void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d>& poses)
{
    ifstream fcamTrans(camTransFile);
    if (!fcamTrans.is_open())
    {
        cerr << "trajectory is empty!" << endl;
        return;
    }
    else
    {
        string str;
        while ((getline(fcamTrans, str)))
        {
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

            if (str.at(0) == '#') {
                cout << "str" << str << endl;
                continue;
            }
            istringstream strdata(str);

            strdata >> t[0] >> t[1] >> t[2] >> q.x() >> q.y() >> q.z() >> q.w();
            T.rotate(q);
            T.pretranslate(t);
            poses.push_back(T);
        }
    }
}
