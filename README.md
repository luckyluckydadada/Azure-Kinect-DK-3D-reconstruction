# Azure-Kinect-DK-3D-reconstruction
```
git clone https://github.com/luckyluckydadada/Azure-Kinect-DK-3D-reconstruction.git
```
## Acquiring
AcquiringImages 将原始深度图转换到RGB摄像头坐标系下，得到的配准后的深度图，并将转换后的depth图，原始RGB图、原始IR图保存在本地，采集格式仿照TUM数据集，图片的命名格式为时间戳+.png后缀。闭终端或终止程序即可结束图像的采集。
参考: https://blog.csdn.net/Zlp19970106/article/details/107120743

AcquiringPointCloud 将原始深度图转换到RGB摄像头坐标系下，得到的配准后的深度图，并将转换后的(带rgb信息)depth图利用Calibration转为点云保存到本地，图片的命名格式为深度图的时间戳+.ply/pcd后缀。闭终端或终止程序即可结束点云的采集。
参考: https://blog.csdn.net/y18771025420/article/details/113468859

### 编译
已在Ubuntu18.04上编译测试通过，需要自行装好opencv和pcl库。
```
cd Acquiring
mkdir build
cd build
cmake .. -GNinja
ninja
```
### 执行
```
mkdir rgb depth ir 
./AcquiringImages
mkdir ply pcd 
./AcquiringPointCloud
```

## open3d_reconstruction
利用开源框架open3d的Reconstruction system实现Azure Kinect DK相机的三维重建。
目前在Ubuntu18.04笔记本+DK相机上录制的数据，可以携带笔记本连着dk相机（充电宝供电给相机）进行室外采集。
目前在win10台式机上进行重建，主要是因为我的笔记本配置较低，重建的时候比较卡。
后续会将在jetson+dk相机上进行，最好可以实时。
请确保您完成了https://blog.csdn.net/weixin_41965898/article/details/116451026和https://blog.csdn.net/weixin_41965898/article/details/116596933 并通过测试，再继续下面的过程。

### 安装

```
git clone https://github.com/luckyluckydadada/Azure-Kinect-DK-3D-reconstruction.git
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
pip install -r requirements.txt
```
### 录制和提取数据
```
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
录制数据，空格开始录制，esc退出录制并保存：
python sensors/azure_kinect_recorder.py --output dataset/name.mkv 
提取rgb和depth图像，以及相机config和相机内参config：
python sensors/azure_kinect_mkv_reader.py --input dataset/name.mkv --output dataset/name  
```
### 进行重建
```
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
如果发生目录移动，请修改dataset/name/config.json 中的"path_dataset" 和 "path_intrinsic"的值:
python run_system.py dataset/name/config.json --make --register --refine --integrate
```
## associate.py
结束图像采集后，利用TUM数据集associate.py将RGB图和深度图相关联.
使用从https://vision.in.tum.de/data/datasets/rgbd-dataset/tools 下载的associate.py处理rgb.txt和depth.txt，生成配对的文件associate.txt
### 使用
```
python2 associate.py AcquiringImages/build/rgb.txt AcquiringImages/build/depth.txt > AcquiringImages/build/associate.txt
```
