#include "image_matcher.h"
#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils/useful_function.h"
#include "zmq/zmq_frame.h"
#include <math.h>

#ifdef _IMAGE_MATCHER_DEBUG_SHOW
#include <opencv2/highgui/highgui.hpp>
#endif

#define TEST_SEARCH_ 0
#define MATCH_USE_CROSS_CHECK_ 1
#define TEST_TIME_ 1

//类的构造函数
KeyPointImageMatcher::KeyPointImageMatcher(const cv::Mat K,
                                           const cv::Mat discoff,
                                           double init_T_c2geo_tq[])
{
    mbAcceptImgdata=true;
    mnGeoId=0;
    Config kpt_img_matcher_config;
    K.convertTo(kpt_img_matcher_config.K_,CV_64F);
    discoff.convertTo(kpt_img_matcher_config.discoff_,CV_64F);
    // // 图像匹配的位姿初值，认为SLAM初始化用到的两张图像在该位姿附近:190s时

    // 图像匹配的位姿初值
    Eigen::Vector3d init_T_c2geo_t(init_T_c2geo_tq[0],init_T_c2geo_tq[1],init_T_c2geo_tq[2]);
    Eigen::Quaterniond init_T_c2geo_q(init_T_c2geo_tq[6],init_T_c2geo_tq[3] , init_T_c2geo_tq[4],init_T_c2geo_tq[5] );

    // 将图像匹配信息输出到一个txt中
    FILE *fpWriteifo = fopen("Matcher_info.txt", "w");
    fclose(fpWriteifo);

    FILE *fpWriteRefineifo = fopen("RefineGeoTrajectory.txt", "w");
    fclose(fpWriteRefineifo);

    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d init_T_c2geo_R = init_T_c2geo_q.toRotationMatrix();
    // 将旋转部分放入位姿变换矩阵
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            minit_T_c2geo_data[3 * i + i + j] = init_T_c2geo_R(i, j);
        }
    }
    // 将平移向量放入位姿变换矩阵
    minit_T_c2geo_data_xy[0] = init_T_c2geo_t.x();
    minit_T_c2geo_data_xy[1] = init_T_c2geo_t.y();
    minit_T_c2geo_data[3] = 0;
    minit_T_c2geo_data[7] = 0;
    minit_T_c2geo_data[11] = init_T_c2geo_t.z();
}

Sophus::SE3d KeyPointImageMatcher::CvmatToSE3d(cv::Mat cvMat34d)
{
    Eigen::Matrix3d EigenR_tmp,EigenR;
    Eigen::Vector3d Eigent;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            EigenR(i, j) = cvMat34d.at<double>(i, j);
        }
        Eigent(i) = cvMat34d.at<double>(i, 3);
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(EigenR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity(); // 创建一个单位矩阵

    // 使用对角线上的奇异值进行微调
    S.diagonal() << 1, 1, (V * U.transpose()).determinant();

    EigenR_tmp = U * S * V.transpose(); // 微调后的单位正交旋转矩阵

    Sophus::SE3d TSE3d(EigenR_tmp, Eigent);
    return TSE3d;
}

// 插入图像数据,由外部localmapping线程调用;
void KeyPointImageMatcher::InsertImgdata(NewImgdata Imgdata)
{
    std::unique_lock<std::mutex> lock(mMutexNewImgdata);
    // 将图像数据插入到列表中
    mlNewImgdata.push_back(Imgdata);
}

/**
 * @brief 查看列表中是否有等待被插入的图像数据
 */
bool KeyPointImageMatcher::CheckNewImgdata()
{
    std::unique_lock<std::mutex> lock(mMutexNewImgdata);
    return(!mlNewImgdata.empty());
}

void KeyPointImageMatcher::ProcessNewImgdata()
{
    std::unique_lock<std::mutex> lock(mMutexNewImgdata);
    // 取出最前面的图像数据后，在原来的列表里删掉该图像数据
    mCurrentImgdata = mlNewImgdata.front();
    // std::cout << "图像信息id:" << mCurrentImgdata.mnId << std::endl;
    mlNewImgdata.pop_front();
}

void KeyPointImageMatcher::SendRefineImgdata()
{
    std::unique_lock<std::mutex> lock(mMutexRefImgdata);
    // 将图像数据插入到列表中
    mlRefineImgdata.push_back(mCurRefineImgdata);
}


/**
 * @brief 设置"允许接受图像数据"的状态标志
 */
void KeyPointImageMatcher::SetAcceptImgdata(bool flag)
{
    std::unique_lock<std::mutex>  lock(mMutexAccept);
    mbAcceptImgdata=flag;
}

 bool KeyPointImageMatcher::AcceptImgdata()
{
    std::unique_lock<std::mutex>  lock(mMutexAccept);
    return mbAcceptImgdata;
}

void KeyPointImageMatcher::Run()
{
    // 标记状态，表示当前run函数正在运行，尚未结束
    mbFinished = false;
    // 主循环
    while (1)
    {
        // std::cout<<"开启图像匹配线程！"<<std::endl;
        // Step 1 告诉LocalMapping线程，ImageMatcher线程正处于繁忙状态
        SetAcceptImgdata(false);

        // Check if there are keyframes in the queue
        // 等待处理的关键帧列表不为空 并且imu正常
        if (CheckNewImgdata())
        {
            mCurrentImgdata = mlNewImgdata.front();
            // 取出最前面的图像数据后，在原来的列表里删掉该关键帧
            mlNewImgdata.pop_front();
            // std::cout<<"图相匹配线程::图像信息id:" << mCurrentImgdata.mnId << std::endl;

            if (!mCurrentImgdata.bInitFinished)
            {
                std::copy(minit_T_c2geo_data, minit_T_c2geo_data + 12, mCurrentImgdata.init_T_c2geo_data);
            }

            cv::Mat init_T_c2geo(3, 4, CV_64F, mCurrentImgdata.init_T_c2geo_data); // 相机到地理坐标系的T
            cv::Mat init_T(3, 4, CV_64F); // 地理坐标系到相机坐标系的T

            cv::Mat init_R_tmp = init_T_c2geo(cv::Rect(0, 0, 3, 3));
            cv::Mat init_t_tmp = -init_T_c2geo(cv::Rect(0, 0, 3, 3)).t() * init_T_c2geo(cv::Rect(3, 0, 1, 3));
            init_R_tmp.copyTo(init_T(cv::Rect(0, 0, 3, 3)));
            init_t_tmp.copyTo(init_T(cv::Rect(3, 0, 1, 3)));
            cv::Mat refined_T;
            int NumMatchPoins;
            int flag = MatchToReferenceSat(mCurrentImgdata.img,
                                cv::Mat::ones(mCurrentImgdata.img.size(), CV_8U),
                                init_T,                               
                                refined_T,
                                100,
                                minit_T_c2geo_data_xy,
                                NumMatchPoins);  

            cv::Mat refined_Tc2g(3, 4, CV_64F);
            cv::Mat refined_R_tmp = refined_T(cv::Rect(0, 0, 3, 3)).t();
            cv::Mat refined_t_tmp = -refined_R_tmp * refined_T(cv::Rect(3, 0, 1, 3));
            


            refined_R_tmp.copyTo(refined_Tc2g(cv::Rect(0, 0, 3, 3)));
            refined_t_tmp.copyTo(refined_Tc2g(cv::Rect(3, 0, 1, 3)));

            std::cout << "图像匹配线程::输出的精确的T:\n"
                      << refined_Tc2g << std::endl;

            Sophus::SE3d Refine_TgcSE3d = CvmatToSE3d(refined_Tc2g);
            Sophus::SE3d Init_TgcSE3d = CvmatToSE3d(init_T_c2geo);

            Eigen::Quaterniond Refine_TgcSE3d_q = Refine_TgcSE3d.unit_quaternion();
            Eigen::Vector3d Refine_TgcSE3d_t = Refine_TgcSE3d.translation();
            FILE *fpWriteRefineinfo = fopen("RefineGeoTrajectory.txt", "a");

            fprintf(fpWriteRefineinfo, "%f %f %f %f %f %f %f %f\n",
                    mCurrentImgdata.sTimeStamp,
                    Refine_TgcSE3d_t(0) + minit_T_c2geo_data_xy[0], Refine_TgcSE3d_t(1) + minit_T_c2geo_data_xy[1], Refine_TgcSE3d_t(2),
                    Refine_TgcSE3d_q.x(), Refine_TgcSE3d_q.y(), Refine_TgcSE3d_q.z(), Refine_TgcSE3d_q.w());
            fclose(fpWriteRefineinfo);
            Sophus::SE3d ErrorSE3d = Refine_TgcSE3d * Init_TgcSE3d.inverse();
            Eigen::Matrix<double, 6, 1> Se3d = ErrorSE3d.log();

            double Se3d_Aver = 0;
            for (int i = 0; i < 2; i++)
            {
                Se3d_Aver += (Se3d(i) * Se3d(i));
            }
            Se3d_Aver = Se3d_Aver / 2;

            FILE *fpWriteifo = fopen("Matcher_info.txt", "a");

            if (fpWriteifo == NULL)
            {
                printf("File Open Failed !\n");
            }

            float belt1 = 4.0 / Se3d_Aver;
            float belt2 = NumMatchPoins / 800.0;
            float beltAll = belt1 + belt2;
            beltAll = sqrt(beltAll);

            fprintf(fpWriteifo, "%d %f %f %f %f %f %f %d %f %f %f\n",
                    flag, ErrorSE3d.log()[0], ErrorSE3d.log()[1], ErrorSE3d.log()[2],
                    ErrorSE3d.log()[3], ErrorSE3d.log()[4], ErrorSE3d.log()[5],
                    NumMatchPoins, belt1, belt2, beltAll);
            fclose(fpWriteifo);

            // std::cout << "图像匹配误差信息：" << flag << " " << ErrorSE3d.log().transpose() << " " << NumMatchPoins << std::endl;
            // std::cout << "图像匹配前后误差：" << Se3d_Aver << std::endl;

            // 如果图像匹配成功，将匹配结果发送到图像匹配线程
            if (flag == 1)
            {
                std::cout << "图像匹配线程::发送图像数据！"<< std::endl;
                refined_T.copyTo(mCurRefineImgdata.RefineT);
                mCurRefineImgdata.mnId = mCurrentImgdata.mnId;
                mCurRefineImgdata.mnGeoId = mnGeoId;
                if (!mCurrentImgdata.bInitFinished)
                    mCurRefineImgdata.Information = 1;
                else
                    mCurRefineImgdata.Information = beltAll;
                SendRefineImgdata();
                mnGeoId++;
                std::cout << "图像匹配线程::地理定位图像Id:"<<  mCurRefineImgdata.mnGeoId << std::endl;
            }
        }
        // 开始接收图像数据
        SetAcceptImgdata(true);
        usleep(3000);
    }
}

