#include <iostream>
#include "ccpp.h"

std::vector<ccpp_planner::MapPose> edge_path;
bool get_edge_path_flag = false;
void getEdgePath(int event, int x, int y, int flags, void*)
{
    if (/*event == CV_EVENT_MOUSEMOVE &&*/ (flags & cv::EVENT_FLAG_LBUTTON))  //左击
    {
        cv::Point pt1 = cv::Point(x, y);

        ccpp_planner::MapPose point_;
        point_.x = pt1.x;
        point_.y = pt1.y;
        edge_path.push_back(point_);

        get_edge_path_flag = false;
    }
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        get_edge_path_flag = true;
    }
}

int main()
{
    std::cout << "Hello, World!" << std::endl;

    cv::Mat room_map = cv::imread("../maps/sz_office_4f.png",CV_8UC1);
    //cv::Mat room_map = cv::imread("../maps/test_ccpp_4th.png",CV_8UC1);
    if(room_map.empty())
    {
        std::cerr<<"Can't open the map png image"<<std::endl;
        return -1;
    }

    //显示
    cv::Mat display_img;
    room_map.copyTo(display_img);
    cv::Mat clc_img;
    room_map.copyTo(clc_img);
    cv::Mat wall_display_img;
    room_map.copyTo(wall_display_img);
    cv::namedWindow("display_img",0);
    //cv::namedWindow("wall_display_img",0);
    printf("左键点击鼠标，确定规划区域顶点， \n");
    printf("右键点击鼠标，开始区域规划任务， \n");

    while(1)
    {
        //获取规划起始点
        cv::setMouseCallback("display_img",getEdgePath,0);
        cv::waitKey(1);

        if(get_edge_path_flag)
        {
            get_edge_path_flag = false;

            ccpp_planner::CCPP b_ccpp;
            std::vector<ccpp_planner::MapPose> ccpp_path;
            std::vector<ccpp_planner::MapPose> wall_path;
            b_ccpp.runCCPP(room_map, edge_path, ccpp_path, wall_path);

            for(int i=0; i<ccpp_path.size(); i++)
            {
                cv::circle(display_img,cv::Point(ccpp_path[i].x,ccpp_path[i].y),2,cv::Scalar(128),-1);
            }
            cv::imshow("display_img",display_img);
            //cv::waitKey(0);

            for(int j=0; j<wall_path.size();j++)
            {
                cv::circle(wall_display_img,cv::Point(wall_path[j].x,wall_path[j].y),2,cv::Scalar(128),-1);
            }
            cv::imshow("wall_path_display_img",wall_display_img);
            cv::waitKey(0);

            edge_path.clear();
        }

        clc_img.copyTo(display_img);
        clc_img.copyTo(wall_display_img);
        for(int i=0; i<edge_path.size(); i++)
        {
            cv::circle(display_img,cv::Point(edge_path[i].x,edge_path[i].y),2,cv::Scalar(128),-1);
            cv::circle(wall_display_img,cv::Point(edge_path[i].x,edge_path[i].y),2,cv::Scalar(128),-1);
        }
        cv::imshow("display_img",display_img);
        cv::imshow("wall_display_img",wall_display_img);
        cv::waitKey(1);
    }

    return 0;
}
