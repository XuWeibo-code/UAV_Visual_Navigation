/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>



using namespace std;


cv::Mat Matrix2Quaternion(cv::Mat matrix) {
    float tr, qx, qy, qz, qw;

    // 计算矩阵轨迹
    float a[4][4] = {0};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
        a[i][j]=matrix.at<float>(i,j);
    
    // I removed + 1.0f; see discussion with Ethan
    float trace = a[0][0] + a[1][1] + a[2][2]; 
    if( trace > 0 ) {
        // I changed M_EPSILON to 0
        float s = 0.5f / sqrtf(trace+ 1.0f);
        qw = 0.25f / s;
        qx = ( a[2][1] - a[1][2] ) * s;
        qy = ( a[0][2] - a[2][0] ) * s;
        qz = ( a[1][0] - a[0][1] ) * s;
    } else {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
        float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
        qw = (a[2][1] - a[1][2] ) / s;
        qx = 0.25f * s;
        qy = (a[0][1] + a[1][0] ) / s;
        qz = (a[0][2] + a[2][0] ) / s;
        } else if (a[1][1] > a[2][2]) {
        float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
        qw = (a[0][2] - a[2][0] ) / s;
        qx = (a[0][1] + a[1][0] ) / s;
        qy = 0.25f * s;
        qz = (a[1][2] + a[2][1] ) / s;
        } else {
        float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
        qw = (a[1][0] - a[0][1] ) / s;
        qx = (a[0][2] + a[2][0] ) / s;
        qy = (a[1][2] + a[2][1] ) / s;
        qz = 0.25f * s;
        }    
    }

    float q[] = {qx,qy,qz, qw};
    //cout<< "\n quaternion:"<<cv::Mat(4,1,CV_32FC1,q).t()<<endl;
    return cv::Mat(4,1,CV_32FC1,q).clone();
}

int main(int argc, char **argv)
{  
    if(argc < 4) {
        cerr << endl << "Usage: ./mono_vedio path_to_vocabulary path_to_settings vedio" << endl;
        return 1;
    }

    // read trajectory file
    // TODO:读取gps数据,并转换为投影坐标系
    // vector<cv::Point3f> gr_ts;
    // ifstream fin;
    // fin.open(argv[5]);
    // if(!fin.is_open()){
    //     std::cerr << "Cannot open file: " << std::string(argv[5]) << std::endl;
    //     exit(-1);
    // }
    // fin.close();

    //float v_scale = atof(argv[4]);

    // open video
    cv::VideoCapture cap;
    cap.open(argv[3]);
    if(!cap.isOpened()){
        std::cerr << "Cannot open video: " << string(argv[3]) << endl;
        exit(-1);
    }
    cv::Mat cur_img, cur_img_scaled;
    //cap >> cur_img;
    //cv::imshow("cur_img", cur_img);

    // output result
    ofstream gr_out, es_out;
    // gr_out.open("gr_traj.txt");
    es_out.open("es_traj.txt");

    // count frame to compute time
    // double dt = 0.0333;
    double dt = 0.05;
    double img_num = 0;
    double cur_t;
    Sophus::SE3f cur_pose;

    // std::ofstream fout("trajectory_es.txt");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    double t_start_start = cv::getTickCount(); // 开始处理图像的时间
    bool First_flag = 1;
    while (true)
    {
        double t_start = cv::getTickCount();
        cap >> cur_img;
        //cv::imshow("cur_img", cur_img);
        //cout << "图像宽为" << cur_img.cols << ",高为" << cur_img.rows << ",通道数为" << cur_img.channels() << endl;
        //double t_used1 = (cv::getTickCount() - t_start) / cv::getTickFrequency();
        //std::cout <<"cv::VideoCapture读取图片用了"<< t_used1 * 1000 << " ms\n";



        if(cur_img.empty())
        {
            cout<<"找不到图片！"<<endl;
            break;
        }

        // cv::resize(cur_img, cur_img_scaled, cv::Size(), v_scale, v_scale);
        cur_t = img_num * dt;
        ++img_num;

        double t_now1 = (cv::getTickCount() - t_start_start) / cv::getTickFrequency();
        // std::cout << "输入帧时间: " << cur_t << " s \n";
        //std::cout << "当前时间: " << t_now1 << " s \n";

        // First_flag确保能读到第一帧
        if (cur_t < t_now1 + 0.00001 && First_flag == 0)
        {
            cout << "丢弃该帧:" << cur_t << "s" << endl;
            continue;
        }

        if (First_flag == 1)
        {
            cout << "接收第一帧！" << endl;
            //cv::imwrite("FirstCap.png",  cur_img);
            First_flag = 0;
        }
        // fake image
        // if(int(img_num) / 200 % 10 == 0){
        // // if(img_num>1000 && img_num<2000){
        //     cur_img_scaled = cv::Mat::zeros(cur_img_scaled.size(), cur_img_scaled.type());
        // }

        cur_pose = SLAM.TrackMonocular(cur_img, cur_t);

        double t_used2 = (cv::getTickCount() - t_start) / cv::getTickFrequency();
        //std::cout << "图片处理时间： " << t_used2 * 1000 << " ms\n";

        auto cur_pose_mat = cur_pose.matrix();
        //fout << cur_pose_mat(0, 3) << " " << cur_pose_mat(1, 3) << " " << cur_pose_mat(2, 3) << "\n";
        // std::cout << "cur_pose: " <<std::endl<< cur_pose.matrix() << std::endl;
        // std::cout << "相机坐标系到世界坐标系转换矩阵:\n" << cur_pose_mat.inverse()<<std::endl<<std::endl;
        //std::cout << "Dealing with frame: " << img_num << std::endl;
        //std::cout << "current frame tracking status: " << SLAM.GetTrackingState() << std::endl;
        //double t_used = (cv::getTickCount() - t_start) / cv::getTickFrequency();
        //std::cout << "SLAM处理当前帧用了"<<t_used * 1000 << " ms "<<endl;
        double t_current = (cv::getTickCount() - t_start_start) / cv::getTickFrequency();
       //std::cout << "当前时间： " << t_current << " s "<<endl;

       double t_now2 = (cv::getTickCount() - t_start_start) / cv::getTickFrequency();

       int sleep_time = ( cur_t+dt )* 1000 - t_now2 * 1000-4;



       //int sleep_time = dt * 1000 - t_used2 * 1000;
      //std::cout << "需要休眠" << sleep_time << " ms "<<endl;

        
        if(sleep_time>0)
        {
            //std::cout << "休眠了" << sleep_time << " ms "<<endl;
             usleep(sleep_time*1000);
        }

        // cout<<endl;
           
/*
        if(cur_pose_mat(0,0)==0){
            std::cout << "empty pose...\n";
            continue;
        }
*/

        // 输出估计轨迹
        cv::Mat cur_pose_cvMat(4,4,CV_32F);
        for(int i=0; i<16; ++i){
            cur_pose_cvMat.at<float>(i/4, i%4) = cur_pose_mat(i/4, i%4);
        }
        // std::cout<<"cur_pose_cvMat:\n"<<cur_pose_cvMat<<std::endl;
        cv::Mat R, t;
        R = cur_pose_cvMat(cv::Rect(0, 0, 3, 3));
        t = cur_pose_cvMat(cv::Rect(3, 0, 1, 3));
        R = R.t();
        t = -R * t;
        // std::cout<<"R:\n"<<R<<std::endl;
        cv::Mat q = Matrix2Quaternion(R);
        es_out << setiosflags(ios::fixed) << setprecision(6) << cur_t << " "
               << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
               << q.at<float>(0) << " " << q.at<float>(1) << " " << q.at<float>(2) << " " << q.at<float>(3) << endl;

        // gr_out<< cur_t << " "
        //        << gr_ts[img_num].x << " " << gr_ts[img_num].y << " " << gr_ts[img_num].z << " "
        //        << 0 << " " << 0 << " " << 0 << " " << 1 << endl;
    }

    //fout.close();
    SLAM.Shutdown();

    SLAM.SaveTrajectoryGeo("GeoTrajectory.txt");
    SLAM.SaveTrajectoryGeoz0("GeoTrajectory_z0.txt");

    //SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
   

    return 0;
}
