
#include <iostream>
#include <chrono>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Kinect DK
#include <k4a/k4a.hpp>

using namespace cv;
using namespace std;

// 方便控制是否 std::cout 信息
#define DEBUG_std_cout 0


int main(int argc, char* argv[]) {
	/*
		找到并打开 Azure Kinect 设备
	*/
	// 发现已连接的设备数

	const uint32_t device_count = k4a::device::get_installed_count();
	if (0 == device_count) {
		std::cout << "Error: no K4A devices found. " << std::endl;
		return -1;
	}
	else {
		std::cout << "Found " << device_count << " connected devices. " << std::endl;
		if (1 != device_count)// 超过1个设备，也输出错误信息。
		{
			std::cout << "Error: more than one K4A devices found. " << std::endl;
			return -1;
		}
		else// 该示例代码仅限对1个设备操作
		{
			std::cout << "Done: found 1 K4A device. " << std::endl;
		}
	}
	// 打开（默认）设备
	k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
	std::cout << "Done: open device. " << std::endl;

	/*
		检索并保存 Azure Kinect 图像数据
	*/
	// 配置并启动设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	//config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
	device.start_cameras(&config);
	std::cout << "Done: start camera." << std::endl;

    //写入txt文件流
    ofstream rgb_out;
    ofstream d_out;
    ofstream ir_out;
    
    rgb_out.open("./rgb.txt");
    d_out.open("./depth.txt");
    ir_out.open("./ir.txt");
    
    rgb_out<<"#  color images"<<endl;
    rgb_out<<"#  file: rgbd_dataset"<<endl;
    rgb_out<<"#  timestamp"<<"    "<<"filename"<<endl;
    
    d_out<<"#  depth images"<<endl;
    d_out<<"#  file: rgbd_dataset"<<endl;
    d_out<<"#  timestamp"<<"    "<<"filename"<<endl;

    ir_out<<"#  ir images"<<endl;
    ir_out<<"#  file: rgbd_dataset"<<endl;
    ir_out<<"#  timestamp"<<"    "<<"filename"<<endl;
    
    rgb_out<<flush;
    d_out<<flush;
    // 稳定化
    k4a::capture capture;
    int iAuto = 0;//用来稳定，类似自动曝光
    int iAutoError = 0;// 统计自动曝光的失败次数
    while (true) {
        if (device.get_capture(&capture)) {
            std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
            
            // 跳过前 n 个（成功的数据采集）循环，用来稳定
            if (iAuto != 30) {
                iAuto++;
                continue;
            } else {
                std::cout << "Done: auto-exposure" << std::endl;
                break;// 跳出该循环，完成相机的稳定过程
            }
            
        } else {
            std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
            if (iAutoError != 30) {
                iAutoError++;
                continue;
            } else {
                std::cout << "Error: failed to give auto-exposure. " << std::endl;
                return -1;
            }
        }
    }
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "----- Have Started Kinect DK. -----" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
     // 从设备获取捕获
    k4a::image rgbImage;
    k4a::image depthImage;
    k4a::image irImage;
    k4a::image transformed_depthImage;
    
    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_irImage;
    cv::Mat cv_irImage_8U;
    
    // for (size_t i = 0; i < 100; i++)
    while (true){        
        // if (device.get_capture(&capture, std::chrono::milliseconds(0)))
        if (device.get_capture(&capture)) {
            // rgb
            // * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
            // * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
            rgbImage = capture.get_color_image();
            #if DEBUG_std_cout == 1
            std::cout << "[rgb] " << "\n"
                << "format: " << rgbImage.get_format() << "\n"
                << "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
                << "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
                << "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels()
                << std::endl;
            #endif

            // depth
            // * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
            // * millimeters from the origin of the camera.
            depthImage = capture.get_depth_image();
            #if DEBUG_std_cout == 1
            std::cout << "[depth] " << "\n"
                << "format: " << depthImage.get_format() << "\n"
                << "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
                << "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
                << "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels()
                << std::endl;
            #endif

            // ir
            // * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
            // * brightness.
            irImage = capture.get_ir_image();
            #if DEBUG_std_cout == 1
            std::cout << "[ir] " << "\n"
                << "format: " << irImage.get_format() << "\n"
                << "device_timestamp: " << irImage.get_device_timestamp().count() << "\n"
                << "system_timestamp: " << irImage.get_system_timestamp().count() << "\n"
                << "height*width: " << irImage.get_height_pixels() << ", " << irImage.get_width_pixels()
                << std::endl;
            #endif
            //深度图和RGB图配准
            k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);// Get the camera calibration for the entire K4A device, which is used for all transformation functions.
            k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
            transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);
            cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
                                             (void *) rgbImage.get_buffer());
            cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);
            cv_depth = cv::Mat(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
                               (void *) transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));
            cv_depth.convertTo(cv_depth_8U, CV_8U, 1);
            cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U,
                                 (void *) irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));
            cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);
            
            // show image
            cv::imshow("color", cv_rgbImage_no_alpha);
            cv::imshow("depth", cv_depth_8U);
            cv::imshow("ir", cv_irImage_8U);
            // save image
            double time_rgb = static_cast<double >(std::chrono::duration_cast<std::chrono::microseconds>(
                    rgbImage.get_device_timestamp()).count());

            std::string filename_rgb = std::to_string(time_rgb/1000000)+".png";
            double time_d = static_cast<double >(std::chrono::duration_cast<std::chrono::microseconds>(
                   depthImage.get_device_timestamp()).count());
                  
            std::string filename_d = std::to_string(time_d/1000000)+".png";
           
            double time_ir = static_cast<double >(std::chrono::duration_cast<std::chrono::microseconds>(
                    irImage.get_device_timestamp()).count());
            std::string filename_ir = std::to_string(time_ir/1000000)+".png";
            imwrite("./rgb/"+filename_rgb, cv_rgbImage_no_alpha);
            imwrite("./depth/"+filename_d,cv_depth_8U);
            imwrite("./ir/"+filename_ir, cv_irImage_8U);
            
            std::cout<<"Acquiring!"<<endl;
             
            //写入depth.txt,rgb.txt文件
            rgb_out<<std::to_string(time_rgb/1000000)<<"    "<<"rgb/"<<filename_rgb<<endl;
            d_out<<std::to_string(time_d/1000000)<<"    "<<"depth/"<<filename_d<<endl;
            ir_out<<std::to_string(time_ir/1000000)<<"    "<<"ir/"<<filename_ir<<endl;
            
            rgb_out<<flush;
            d_out<<flush;
            ir_out<<flush;
            cv_rgbImage_with_alpha.release();
            cv_rgbImage_no_alpha.release();
            cv_depth.release();
            cv_depth_8U.release();
            cv_irImage.release();
            cv_irImage_8U.release();

            capture.reset();
            // if (cv::waitKey(0) == 'q') {//按键采集，用户按下'q',跳出循环,结束采集
            //     std::cout << "----------------------------------" << std::endl;
            //     std::cout << "------------- closed -------------" << std::endl;
            //     std::cout << "----------------------------------" << std::endl;
            //     break;
            // }
        } else {
            std::cout << "false: K4A_WAIT_RESULT_TIMEOUT." << std::endl;
        }
    }
    cv::destroyAllWindows();
    rgb_out<<flush;
    d_out<<flush;
    ir_out<<flush;
    rgb_out.close();
    d_out.close();
    ir_out.close();
    
    // 释放，关闭设备
    rgbImage.reset();
    depthImage.reset();
    irImage.reset();
    capture.reset();
    device.close();

    return 1;
}