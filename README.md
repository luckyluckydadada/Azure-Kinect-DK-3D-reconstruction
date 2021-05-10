# Azure-Kinect-DK-3D-reconstruction

## AcquiringImages
AcquiringImages将原始深度图转换到RGB摄像头坐标系下，得到的配准后的深度图，并将转换后的depth图，原始RGB图、原始IR图保存在本地，采集格式仿照TUM数据集，图片的命名格式为时间戳+.png后缀。闭终端或终止程序即可结束图像的采集。
参考: https://blog.csdn.net/Zlp19970106/article/details/107120743

cd AcquiringImages
mkdir build
cd build
mkdir rgb depth ir 
cmake .. -GNinja
ninja
./AcquiringImages

## associate.py
结束图像采集后，利用TUM数据集associate.py将RGB图和深度图相关联.
使用从https://vision.in.tum.de/data/datasets/rgbd-dataset/tools 下载的associate.py处理rgb.txt和depth.txt，生成配对的文件associate.txt
使用方法：
python2 associate.py AcquiringImages/build/rgb.txt AcquiringImages/build/depth.txt > AcquiringImages/build/associate.txt

## AcquiringPointCloud

参考: https://blog.csdn.net/y18771025420/article/details/113468859

cd AcquiringPointCloud
mkdir build
cd build
rm -rf *; cmake .. -GNinja; 
rm -rf  rgb depth ir ply pcd; mkdir rgb depth ir ply pcd ;ninja
./AcquiringPointCloud