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
开发环境是在Ubuntu18.04笔记本+DK相机上进行，可以携带笔记本连着dk相机（充电宝供电给相机）进行室外采集，采集和重建使用的open3d版本不一样，最好用conda建立两个独立的环境。
另外在win10上可以进行重建，但是利用win10+相机采集数据有些小问题，需要自行解决。
后续会将在jetson+dk相机上进行，最好可以实时。
请确保您完成了https://blog.csdn.net/weixin_41965898/article/details/116451026和https://blog.csdn.net/weixin_41965898/article/details/116596933 并通过测试，再继续下面的过程。

### 安装
```
git clone https://github.com/luckyluckydadada/Azure-Kinect-DK-3D-reconstruction.git
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
```
### 录制和提取数据
```
pip install -r requirements-0.10.txt  # 此处安装 open3d版本为 0.10.0
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
录制数据，空格开始录制，esc退出录制并保存：
python sensors/azure_kinect_recorder.py --output dataset/name.mkv 
提取rgb和depth图像，以及相机config和相机内参config：
python sensors/azure_kinect_mkv_reader.py --input dataset/name.mkv --output dataset/name  
```
### 进行重建
```
pip install -r requirements.txt  # 此处安装 open3d版本为 0.12.0
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction
如果发生目录移动，请修改dataset/name/config.json 中的"path_dataset" 和 "path_intrinsic"的值:
python run_system.py dataset/name/config.json --make --register --refine --integrate
```

## open3d_reconstruction ARM 支持
忽略注释
### （二选一）
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction/arm
git clone --recursive -b v0.12.0 https://github.com/intel-isl/Open3D.git src
cd src
git submodule update --init --recursive
mkdir build
<!-- ### （二选一）
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction/arm
wget 
tar -xvf  -->
### 安装依赖
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction/arm
bash install-deps.sh 
重启jetson 
### 编译
cd Azure-Kinect-DK-3D-reconstruction/open3d_reconstruction/arm/build
<!-- 建立virtualenv，并在virtualenv中安装高版本cmake，比下载高版本cmake通过源码编译的方式快
pip3 install virtualenv 
重启
virtualenv --python=$(which python3) ${HOME}/o3d12
source ${HOME}/o3d12/bin/activate -->
pip install cmake # 升级cmake
hash -r
bash 

### 安装Open3D python包(optional) 
因为pip install matplotlib在arm上装不上，用apt安装。
sudo apt install python3-matplotlib
make install-pip-package -j$(nproc)
python3 -c "import open3d; print(open3d)"

# Run Open3D GUI (optional, available on when -DBUILD_GUI=ON)
./bin/Open3D/Open3D


## associate.py
结束图像采集后，利用TUM数据集associate.py将RGB图和深度图相关联.
使用从https://vision.in.tum.de/data/datasets/rgbd-dataset/tools 下载的associate.py处理rgb.txt和depth.txt，生成配对的文件associate.txt
### 使用
```
python2 associate.py AcquiringImages/build/rgb.txt AcquiringImages/build/depth.txt > AcquiringImages/build/associate.txt
```
