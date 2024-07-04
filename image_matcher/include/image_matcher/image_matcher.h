#ifndef H_SLAM_IMAGE_MATCHER_H_
#define H_SLAM_IMAGE_MATCHER_H_

#include "data_structure/tile_map.h"
#include "zmq/zmq_IPC.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <memory.h>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include "utils/useful_function.h"
#include <list>
#include <mutex>
#include <sophus/se3.hpp>

// #include "LocalMapping.h"


#define _IMAGE_MATCHER_DEBUG_SHOW 1

#define _IMAGE_RESULT_SHOW 0




// localmapping和imagematcher线程均可访问这两个公共结构
struct NewImgdata
{
    double sTimeStamp;            //当前关键帧的时间戳
    cv::Mat img;                  // 彩色航拍图
    long unsigned int mnId;       // 航拍图对应的关键帧id，用于图像地理定位的存取
    double init_T_c2geo_data[12]; // GSLAM初始化时这个数组等于minit_T_c2geo_data，随后完全由img的世界坐标转换所得
    bool bInitFinished;
};

struct RefineImgdata
{
    long unsigned int mnId; // 航拍图对应的关键帧id，用于图像地理定位的存取
    int mnGeoId;
    cv::Mat RefineT;        // 经过图像匹配程序得到的地理定位信息
    double Information;
};

class KeyPointImageMatcher
{
public:
    KeyPointImageMatcher(){};

    // 下面的代码用于辅助VO定位
    // 新建的构造函数，配合run函数使用
    KeyPointImageMatcher(const cv::Mat K,
                         const cv::Mat discoff,
                         double init_T_c2geo_tq[]);

    Sophus::SE3d CvmatToSE3d(cv::Mat cvMat34d);

    double minit_T_c2geo_data[12];   // 用于GSLAM地理初始化时赋值的相机位姿，其中xy均为0需要使用两次。
    double minit_T_c2geo_data_xy[2]; // 用于GLAM初始的xy的存储，，避免数值过大造成的不好影响

    std::mutex mMutexNewImgdata;
    std::mutex mMutexRefImgdata;

    std::mutex mMutexAccept;
    NewImgdata mCurrentImgdata;
    RefineImgdata mCurRefineImgdata;

    std::list<NewImgdata> mlNewImgdata;       // 用于存储由localmapping线程输入的图像数据
    std::list<RefineImgdata> mlRefineImgdata; // 用于存储输出到localmapping线程的图像数据

    void InsertImgdata(NewImgdata Imgdata);
    bool CheckNewImgdata();
    void ProcessNewImgdata();
    void SendRefineImgdata();
    int mnGeoId;

    void SetAcceptImgdata(bool flag);
    bool AcceptImgdata();


    void Run();

    // 所有长度单位都为m
    struct Config
    {
        // 确定匹配图的padding的比例系数
        float search_radius_ratio_ = 1.0;

        // 阈值
        float search_radius_threshold_ = 100;

        // 形变模型：0 --- 相似；1 --- 仿射；2 --- 单应; 3 --- PNP
        int transformation_model_ = 0;

        // 相机内参数、畸变参数
        cv::Mat K_;
        cv::Mat discoff_;

        std::string tile_sat_info_file_;
        std::string debug_dir_;
        std::string debug_dir_query;
        std::string result_dir_;

        void PrintInfo()
        {
            LOG(INFO) << "[KeyPointImageMatcher::Config] info:";
            LOG(INFO) << "    transformation_model_ = " << transformation_model_;
            LOG(INFO) << "    search_radius_ratio_ = " << search_radius_ratio_;
            LOG(INFO) << "    tile_sat_info_file_ = " << tile_sat_info_file_;
            LOG(INFO) << "    debug_dir_ = " << debug_dir_;
            LOG(INFO) << "    result_dir_ = " << result_dir_;
            LOG(INFO) << "camera K:\n"
                      << K_;
            LOG(INFO) << "camera discoff: "
                      << discoff_.t();
        }
    };   

private:
    // 参考数据库
    // 栅格地图
    std::shared_ptr<TileMap>
        ref_sat_tile_map_;

    /*************************************状态量************************************/
    int align_num_ = 0;
 
    /************************************* 算法参数 ************************************/
    Config config_;
    
    bool mbFinished;
    bool mbAcceptImgdata;
};

#endif