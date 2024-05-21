#include "ccpp.h"

namespace ccpp_planner
{
    CCPP::CCPP()
    {
        this->coverage_direction = CoverageDirection::HORIZONTAL; // coverage direction
        this->coverage_start_direction = CoverageStartDirection::LEFTUP; // coverage start direction

        this->map_resolution = 0.05;                          // [m]
        this->coverage_resolution = 0.1;                      // [m]
        this->robot_radius = 0.2;                            // [m]
        this->coverage_interval_distance = robot_radius*2*2;  // [m]
        this->inflation_distance = robot_radius + 0.05;        // [m]
        this->path_eps = 1.0; // [pixel],for downsample path,the distance between points when generating a path
        this->predict_times = coverage_interval_distance/coverage_resolution*1; // [pixel],predict distance on map
        this->error_distance = 3;     // [pixel],error distance for turn circle ccpp
        this->ccpp_map_origin_x = 0;  // [pixel],ccpp map origin x value on map
        this->ccpp_map_origin_y = 0;  // [pixel],ccpp map origin y value on map
    }

    CCPP::~CCPP(){}

    void CCPP::configCCPP(CcppConfig config)
    {
        coverage_direction = config.coverage_direction; // coverage direction
        coverage_start_direction = config.coverage_start_direction; 

        map_resolution = config.map_resolution;                          // [m]
        coverage_resolution = config.coverage_resolution;                      // [m]
        robot_radius = config.robot_radius;                            // [m]
        coverage_interval_distance = config.coverage_interval_distance;
        inflation_distance = config.inflation_distance;        // [m]
        path_eps = config.path_eps; 
        predict_times = config.predict_times;
        error_distance = config.error_distance; 
        ccpp_map_origin_x = config.ccpp_map_origin_x;
        ccpp_map_origin_y = config.ccpp_map_origin_y; 

        // TODO: cleanup the config with "this" pointer
        config_ = config;
    }

    bool CCPP::initCCPP(const cv::Mat& map,
                                     const std::vector<MapPose>& edge_path,
                                     cv::Mat& ccpp_map,
                                     cv::Mat& inflation_map)
    {
        if(edge_path.empty())
        {
            std::cout<<"edge path is empty"<<std::endl;
            return false;
        }

        // get ccpp map
        getCoverageMap(map, edge_path, ccpp_map);

        cv::threshold(ccpp_map, ccpp_map, 200, 255, cv::THRESH_BINARY);
        // 膨胀
        int expend_distance_in_pixel = int(this->inflation_distance / this->map_resolution);
        cv::erode(ccpp_map, ccpp_map, cv::Mat(), cv::Point(-1, -1), expend_distance_in_pixel);

        // 闭操作
        cv::Mat temp;
        cv::Mat rotated_room_map;
        cv::erode(ccpp_map, temp, cv::Mat(), cv::Point(-1, -1), this->robot_radius/this->map_resolution );
        cv::dilate(temp, rotated_room_map, cv::Mat(), cv::Point(-1, -1), this->robot_radius/this->map_resolution );

        int fail_status =  this->room_rotation.removeUnconnectedRoomParts(rotated_room_map);
        if(fail_status)
        {
            std::cout<<"no path"<<std::endl;
            return false;
        }

        /// rotate room map
        cv::Rect bbox;
        this->room_rotation.computeRoomRotationMatrix(rotated_room_map, R_, bbox, map_resolution, 0, 0.0);
        this->room_rotation.rotateRoom(rotated_room_map, ccpp_map, R_, bbox);

        // resize
        int new_cols = ccpp_map.cols * this->map_resolution / this->coverage_resolution;
        int new_rows = ccpp_map.rows * this->map_resolution / this->coverage_resolution;
        cv::resize(ccpp_map, ccpp_map, cv::Size(new_cols, new_rows));

        // 二值化map
        cv::threshold(ccpp_map,ccpp_map,200,255,cv::THRESH_BINARY);

        // get coverage direction
        this->coverage_direction = getCoverageDirection(ccpp_map);

        // if coverage direction is vertical then transpose the map
        if(this->coverage_direction == CoverageDirection::VERTICAL)
        {
           cv::transpose(ccpp_map, ccpp_map);
           std::cout << "coverage direction is Vertical" << std::endl;
        }

        // get astar planner map
        ccpp_map.copyTo(inflation_map);

        // added by wangjiajia
        generator_.setWorldData(inflation_map, false, 0, this->coverage_resolution, 30);
        // end added

        // find start pose
        bool find_first_start_pose_status = findStartPose(ccpp_map, this->start_pose);
        if(!find_first_start_pose_status)
        {
            std::cout << "can't find first start pose" << std::endl;
            return false;
        }
        std::cout << "find first start pose : "<< this->start_pose.x << "," << this->start_pose.y << std::endl;
        // add start pose to ccpp path
        this->ccpp_path.push_back(this->start_pose);

        return true;
    }

    bool CCPP::runCCPP(const cv::Mat& map,
                       const std::vector<MapPose>& edge_path,
                       std::vector<MapPose>& output_ccpp_path,
                       std::vector<MapPose>& output_wall_path)
    {
        clock_t start = clock();

        cv::Mat ccpp_map, inflation_map;
        /// ccpp init
        bool init_status = initCCPP(map, edge_path, ccpp_map, inflation_map);
        if(!init_status) {
            return false;
        }

        /// update current pose
        current_pose = start_pose;
        current_pose.direction = MapDirection::RIGHT;

        /// get wall_pose_list
        std::vector<std::vector<MapPose>> wall_pose_list;
        std::vector<std::vector<MapPose>> turn_wall_pose_list;
        std::vector<std::vector<MapPose>> contour_wall_pose_list;
        getCoverageWallPose(ccpp_map, this->start_pose,
                            wall_pose_list, turn_wall_pose_list);

        /// Xu's work
        cv::Mat edge_map;
        int expend_distance_in_pixel = int(this->inflation_distance / this->map_resolution);
        cv::erode(ccpp_map, edge_map, cv::Mat(), cv::Point(0,1), expend_distance_in_pixel);
        
        // ref: https://blog.csdn.net/hust_bochu_xuchao/article/details/51918907
        std::vector<std::vector<cv::Point>> wall_following_contours_out;
        cv::findContours(edge_map, wall_following_contours_out, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        
        double max_area = 0;
        int max_area_idx = 0;
        for (int index = wall_following_contours_out.size() - 1; index >= 0; index--)
        {
            double tmp_area = fabs(cv::contourArea(wall_following_contours_out[index]));
            if (tmp_area > max_area)
            {
                max_area_idx = index;
                max_area = tmp_area;
            }
        }

        
        /// debug
        /*
        std::cout << "Debug contour " << std::endl;

        cv::Mat empty_map(edge_map.rows, edge_map.cols, CV_8UC1,255);
        cv::drawContours(empty_map, wall_following_contour, 0, cv::Scalar(0), 2);
        cv::imshow("contour", empty_map);
        cv::waitKey(0);
         */
        for(int i=0; i < wall_following_contours_out[max_area_idx].size(); i++)
        {
            MapPose wall_point;
            wall_point.x = wall_following_contours_out[max_area_idx][i].x;
            wall_point.y = wall_following_contours_out[max_area_idx][i].y;

            raw_wall_path.push_back(wall_point);
        }

        getCoveragePath(ccpp_map, raw_wall_path, output_wall_path);

        /// up to down coverage
        std::cout<<"start up to down coverage"<<std::endl;
        for(int y=0; y<wall_pose_list.size(); y++)
        {
            if((y%2)==0)
            {
                if(wall_pose_list[y].size() >0 && wall_pose_list[y].size() <= 2)
                {
                    /// move right without obstacle
                    getMoveRightPathWithoutObstacle(wall_pose_list[y]);
                }
                else if(wall_pose_list[y].size() > 2)
                {
                    /// move right with obstacle
                    bool get_status = getMoveRightPathWithObstacle(inflation_map,
                                                                   wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }

                if((this->current_pose.direction == MapDirection::DOWN) &&
                   (y < (wall_pose_list.size()-1)))
                {
                    /// right down
                    bool get_status = getRightDownPath(inflation_map, wall_pose_list[y+1]);
                    if(!get_status){
                        return false;
                    }
                }
            }
            else
            {
                if(wall_pose_list[y].size() > 0 && wall_pose_list[y].size() <= 2)
                {
                    /// move left without obstacle
                    getMoveLeftPathWithoutObstacle(wall_pose_list[y]);
                }
                else if(wall_pose_list[y].size() > 2)
                {
                    /// move left with obstacle
                    bool get_status = getMoveLeftPathWithObstacle(inflation_map,
                                                                  wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }

                if((this->current_pose.direction == MapDirection::DOWN) &&
                   (y < (wall_pose_list.size()-1)))
                {
                    /// left down
                    bool get_status = getLeftDownPath(inflation_map, wall_pose_list[y+1]);
                    if(!get_status){
                        return false;
                    }
                }
            }
        }

        /// down to up coverage
        std::cout<<"start down to up coverage"<<std::endl;
        for(int y=(int(turn_wall_pose_list.size())-1); y>=0; y--)
        {
            // special path
            if(y == (int(turn_wall_pose_list.size())-1))
            {
                if(((wall_pose_list.size()-1)%2) == 0)
                {
                    /// right up
                    bool get_status = getRightDownPath(inflation_map, turn_wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }
                else
                {
                    /// left up
                    bool get_status = getLeftDownPath(inflation_map, turn_wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }
            }

            // normal path
            if((y%2)==0)
            {
                if(turn_wall_pose_list[y].size() >0 && turn_wall_pose_list[y].size() <= 2)
                {
                    /// move right without obstacle
                    getMoveRightPathWithoutObstacle(turn_wall_pose_list[y]);
                }
                else if(turn_wall_pose_list[y].size() > 2)
                {
                    /// move right with obstacle
                    bool get_status = getMoveRightPathWithObstacle(inflation_map,
                                                                   turn_wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }

                if((this->current_pose.direction == MapDirection::DOWN) &&
                   (y > 0))
                {
                    /// right down
                    bool get_status = getRightDownPath(inflation_map, turn_wall_pose_list[y-1]);
                    if(!get_status){
                        return false;
                    }
                }
            }
            else
            {
                if(turn_wall_pose_list[y].size() > 0 && turn_wall_pose_list[y].size() <= 2)
                {
                    /// move left without obstacle
                    getMoveLeftPathWithoutObstacle(turn_wall_pose_list[y]);
                }
                else if(turn_wall_pose_list[y].size() > 2)
                {
                    /// move left with obstacle
                    bool get_status = getMoveLeftPathWithObstacle(inflation_map,
                                                                  turn_wall_pose_list[y]);
                    if(!get_status){
                        return false;
                    }
                }

                if((this->current_pose.direction == MapDirection::DOWN) &&
                   (y > 0))
                {
                    /// left down
                    bool get_status = getLeftDownPath(inflation_map, turn_wall_pose_list[y-1]);
                    if(!get_status){
                        return false;
                    }
                }
            }
        }

        std::cout << " CCPP finish" << std::endl;
        std::cout << "CCPP path size : " << this->ccpp_path.size() << std::endl;

        // downsample ccpp_path
        downsamplePath(this->ccpp_path,
                       this->downsample_ccpp_path,
                       this->ccpp_path.front(),
                       this->path_eps);
        std::cout << "downsample CCPP path size : " << downsample_ccpp_path.size() << std::endl;

        clock_t ends = clock();
        std::cout << "Running Time : " << (double)(ends - start) / CLOCKS_PER_SEC * 1000 <<" ms"<< std::endl;

        // output ccpp path
        getCoveragePath(ccpp_map, this->downsample_ccpp_path, output_ccpp_path);

        // display
        if (config_.coverage_display)
        {
            //coverageDisplay(map, output_ccpp_path);
            //wallPoseDisplay(ccpp_map, contour_wall_pose_list);
            coverageDisplay(map, output_ccpp_path);
        }

        downsample_ccpp_path.clear();
        ccpp_path.clear();
        raw_wall_path.clear();
        return true;
    }

    void CCPP::wallPoseDisplay(const cv::Mat& original_map,
                               const std::vector<std::vector<MapPose>>& wall_pose_list)
    {
        cv::Mat display_map;
        original_map.copyTo(display_map);
        for(auto it = wall_pose_list.begin(); it!= wall_pose_list.end(); it++)
        {
            for(int i = 0; i < (*it).size(); i++)
            {
                cv::Point ss;

                ss.x = (*it)[i].x;
                ss.y = (*it)[i].y;

                cv::circle(display_map,ss,2,cv::Scalar(164),-1);

                cv::namedWindow("wall pose CCPP",0);
                cv::imshow("wall pose CCPP",display_map);
                cv::waitKey(1);
            }    

        }

        cv::namedWindow("wall pose CCPP",0);
        cv::imshow("wall pose CCPP",display_map);
        cv::waitKey(1);
    }

    void CCPP::coverageDisplay(const cv::Mat& original_map,
                               const std::vector<MapPose>& output_ccpp_path)
    {
        cv::Mat display_map;
        original_map.copyTo(display_map);
        for(int i=0; i<(output_ccpp_path.size()-1); i++)
        {
            cv::Point ss,ee;

            ss.x = output_ccpp_path[i].x;
            ss.y = output_ccpp_path[i].y;
            ee.x = output_ccpp_path[i+1].x;
            ee.y = output_ccpp_path[i+1].y;

            cv::line(display_map,ss,ee,cv::Scalar(192),1);
            cv::circle(display_map,ss,2,cv::Scalar(64),-1);

            cv::namedWindow("CCPP",0);
            cv::imshow("CCPP",display_map);
            cv::waitKey(2);
        }
        cv::namedWindow("CCPP",0);
        cv::imshow("CCPP",display_map);
        cv::waitKey(1);
    }

    void CCPP::getCoveragePath(const cv::Mat& ccpp_map,
                               const std::vector<MapPose>& downsampled_path,
                               std::vector<MapPose>& output_ccpp_path)
    {
        std::vector<cv::Point> fov_middlepoint_path;
        std::vector<geometry_msgs::Pose2D> path_fov_poses;
        for(int i=0; i<downsampled_path.size(); i++)
        {
            //modified by wangjiajia
            //MapPose correct_pose;
            cv::Point correct_pose;

            if(this->coverage_direction == CoverageDirection::HORIZONTAL)
            {
                if(this->coverage_start_direction == CoverageStartDirection::LEFTDOWN)
                {
                    correct_pose.x = downsampled_path[i].x *
                                     this->coverage_resolution / this->map_resolution;
                    correct_pose.y = (ccpp_map.rows - downsampled_path[i].y) *
                                     this->coverage_resolution / this->map_resolution;
                }
                else if(this->coverage_start_direction == CoverageStartDirection::RIGHTUP)
                {
                    correct_pose.x = (ccpp_map.cols - downsampled_path[i].x) *
                                     this->coverage_resolution / this->map_resolution;
                    correct_pose.y = downsampled_path[i].y *
                                     this->coverage_resolution / this->map_resolution;
                }
                else if(this->coverage_start_direction == CoverageStartDirection::RIGHTDOWN)
                {
                    correct_pose.x = (ccpp_map.cols - downsampled_path[i].x) *
                                     this->coverage_resolution / this->map_resolution;
                    correct_pose.y = (ccpp_map.rows - downsampled_path[i].y) *
                                     this->coverage_resolution / this->map_resolution;
                }
                else
                {
                    correct_pose.x = downsampled_path[i].x *
                                     this->coverage_resolution / this->map_resolution ;
                    correct_pose.y = downsampled_path[i].y *
                                     this->coverage_resolution / this->map_resolution;
                }
            }
            else
            {
                if(this->coverage_start_direction == CoverageStartDirection::LEFTDOWN)
                {
                    correct_pose.x = downsampled_path[i].y *
                                     coverage_resolution / this->map_resolution;
                    correct_pose.y = (ccpp_map.cols - downsampled_path[i].x) *
                                     this->coverage_resolution / this->map_resolution;
                }
                else if(this->coverage_start_direction == CoverageStartDirection::RIGHTUP)
                {
                    correct_pose.x = (ccpp_map.rows - downsampled_path[i].y) *
                                     coverage_resolution / this->map_resolution;
                    correct_pose.y = downsampled_path[i].x *
                                     this->coverage_resolution / this->map_resolution;
                }
                else if(this->coverage_start_direction == CoverageStartDirection::RIGHTDOWN)
                {
                    correct_pose.x = (ccpp_map.rows - downsampled_path[i].y) *
                                     coverage_resolution / this->map_resolution;
                    correct_pose.y = (ccpp_map.cols - downsampled_path[i].x) *
                                     this->coverage_resolution / this->map_resolution ;
                }
                else
                {
                    correct_pose.x = downsampled_path[i].y *
                                     coverage_resolution / this->map_resolution;
                    correct_pose.y = downsampled_path[i].x *
                                     this->coverage_resolution / this->map_resolution;
                }
            }

            fov_middlepoint_path.push_back(correct_pose);
        }
        this->room_rotation.transformPathBackToOriginalRotation(fov_middlepoint_path, path_fov_poses, this->R_);
        for(int i=0; i<path_fov_poses.size(); i++)
        {
            MapPose correct_pose;
            correct_pose.x = path_fov_poses[i].x + this->ccpp_map_origin_x;
            correct_pose.y = path_fov_poses[i].y + this->ccpp_map_origin_y;
            output_ccpp_path.push_back(correct_pose);
        }
    }

    void CCPP::getMoveRightPathWithoutObstacle(const std::vector<MapPose>& x_wall_pose_list)
    {
        // update next pose
        this->next_pose = x_wall_pose_list.back();
        this->next_pose.direction = MapDirection::DOWN;

        // get path
        for(int x=this->current_pose.x; x<this->next_pose.x; x++)
        {
            MapPose x_pose;
            x_pose.x = x;
            x_pose.y = this->current_pose.y;
            this->ccpp_path.push_back(x_pose);
        }

        // update next pose to current pose
        this->current_pose = this->next_pose;
    }

    void CCPP::getMoveLeftPathWithoutObstacle(const std::vector<MapPose>& x_wall_pose_list)
    {
        this->next_pose = x_wall_pose_list.front();
        this->next_pose.direction = MapDirection::DOWN;

        for(int x=this->current_pose.x; x>=this->next_pose.x; x--)
        {
            MapPose x_pose;
            x_pose.x = x;
            x_pose.y = this->current_pose.y;
            this->ccpp_path.push_back(x_pose);
        }

        // update next pose to current pose
        this->current_pose = this->next_pose;
    }

    bool CCPP::getMoveRightPathWithObstacle(const cv::Mat& inflation_map,
                                                         const std::vector<MapPose>& x_wall_pose_list)
    {
        // get current pose index
        int current_pose_index = -1;
        for(int x=0; x<x_wall_pose_list.size(); x++)
        {
            if(this->current_pose.x == x_wall_pose_list[x].x &&
               this->current_pose.y == x_wall_pose_list[x].y)
            {
                current_pose_index = x;
                break;
            }
        }
        // get path
        if(current_pose_index == -1)
        {
            std::cout<<"find move right current pose index fail"<<std::endl;
            return false;
        }

        bool normal_flag = true;
        int next_pose_index = -1;
        while(current_pose_index < int(x_wall_pose_list.size()))
        {
            // update next pose
            if(normal_flag){
                next_pose_index = current_pose_index + 1;
            }
            else{
                next_pose_index += 2;
            }

            if(next_pose_index >= x_wall_pose_list.size()){
                break;
            }

            this->next_pose = x_wall_pose_list[next_pose_index];
            this->next_pose.direction = MapDirection::DOWN;

            if((current_pose_index%2)==0)
            {
                /// without obstacle
                for(int x=this->current_pose.x; x<this->next_pose.x; x++)
                {
                    MapPose x_pose;
                    x_pose.x = x;
                    x_pose.y = this->current_pose.y;
                    this->ccpp_path.push_back(x_pose);
                }

                // update next pose to current pose
                this->current_pose = this->next_pose;
                current_pose_index = next_pose_index;

                normal_flag = true;
            }
            else
            {
                /// with obstacle

                // use astar path planner conected robot current pose and new start pose
                cv::Point start_point, end_point;
                start_point.x = this->current_pose.x;
                start_point.y = this->current_pose.y;
                end_point.x = this->next_pose.x;
                end_point.y = this->next_pose.y;

                // added by wangjiajia
                auto result = generator_.findPath({ start_point.x, start_point.y }, { end_point.x, end_point.y });
                double path_size = result.size();
                // end

                if(path_size > 0)
                {
                    // add astar path to ccpp path
                    for(int i=0; i<result.size(); i++)
                    {
                        MapPose pose;
                        pose.x = result[i].x;
                        pose.y = result[i].y;
                        this->ccpp_path.push_back(pose);
                    }

                    // update current pose
                    this->current_pose = next_pose;
                    current_pose_index = next_pose_index;

                    normal_flag = true;
                }
                else{
                    normal_flag = false;
                    //std::cout<<"astar planner plan fail"<<std::endl;
                }
            }
        }

        return true;
    }

    bool CCPP::getMoveLeftPathWithObstacle(const cv::Mat& inflation_map,
                                                        const std::vector<MapPose>& x_wall_pose_list)
    {
        // get current pose index
        int current_pose_index = -1;
        for(int x=0; x<x_wall_pose_list.size(); x++)
        {
            if(this->current_pose.x == x_wall_pose_list[x].x &&
               this->current_pose.y == x_wall_pose_list[x].y)
            {
                current_pose_index = x;
                break;
            }
        }
        // get path
        if(current_pose_index == -1)
        {
            std::cout<<"find move left current pose index fail"<<std::endl;
            return false;
        }

        bool normal_flag = true;
        int next_pose_index = 0;
        while(current_pose_index >= 0)
        {
            // update next pose
            if(normal_flag){
                next_pose_index = current_pose_index - 1;
            }
            else{
                next_pose_index -= 2;
            }

            if(next_pose_index < 0){
                break;
            }

            this->next_pose = x_wall_pose_list[next_pose_index];
            this->next_pose.direction = MapDirection::DOWN;

            if(!((current_pose_index%2)==0))
            {
                /// without obstacle
                for(int x=this->current_pose.x; x>=this->next_pose.x; x--)
                {
                    MapPose x_pose;
                    x_pose.x = x;
                    x_pose.y = this->current_pose.y;
                    this->ccpp_path.push_back(x_pose);
                }

                // update next pose to current pose
                this->current_pose = this->next_pose;
                current_pose_index = next_pose_index;

                normal_flag = true;
            }
            else
            {
                /// with obstacle

                // use astar path planner conected robot current pose and new start pose
                cv::Point start_point, end_point;
                start_point.x = this->current_pose.x;
                start_point.y = this->current_pose.y;
                end_point.x = this->next_pose.x;
                end_point.y = this->next_pose.y;

                // added by wangjiajia
                auto result = generator_.findPath({ start_point.x, start_point.y }, { end_point.x, end_point.y });
                double path_size = result.size();
                // end

                if(path_size > 0)
                {
                    // add astar path to ccpp path
                    for(int i=0; i<result.size(); i++)
                    {
                        MapPose pose;
                        pose.x = result[i].x;
                        pose.y = result[i].y;
                        this->ccpp_path.push_back(pose);
                    }

                    // update current pose
                    this->current_pose = next_pose;
                    current_pose_index = next_pose_index;

                    normal_flag = true;
                }
                else{
                    normal_flag = false;
//                    std::cout<<"astar planner plan fail"<<std::endl;
                }
            }
        }

        return true;
    }

    bool CCPP::getRightDownPath(const cv::Mat& inflation_map,
                                             const std::vector<MapPose>& next_x_wall_pose_list)
    {
        // use astar path planner conected robot current pose and new start pose
        int next_pose_index = next_x_wall_pose_list.size()-1;
        while(1)
        {
            this->next_pose = next_x_wall_pose_list[next_pose_index];
            this->next_pose.direction = MapDirection::LEFT;

            cv::Point start_point, end_point;
            start_point.x = this->current_pose.x;
            start_point.y = this->current_pose.y;
            end_point.x = this->next_pose.x;
            end_point.y = this->next_pose.y;

            // added by wangjiajia
            auto result = generator_.findPath({ start_point.x, start_point.y }, { end_point.x, end_point.y });
            double path_size = result.size();
            // end

            if(path_size > 0)
            {
                // add astar path to ccpp path
                for(int i=0; i<result.size(); i++)
                {
                    MapPose pose;
                    pose.x = result[i].x;
                    pose.y = result[i].y;
                    this->ccpp_path.push_back(pose);
                }

                // update next pose to current pose
                this->current_pose = this->next_pose;

                break;
            }
            else
            {
                //std::cout<<"astar planner plan fail"<<std::endl;

                next_pose_index = next_pose_index - 2;
                if(next_pose_index < 0)
                {
                    std::cout<<"connected right next cols fail"<<std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    bool CCPP::getLeftDownPath(const cv::Mat& inflation_map,
                                            const std::vector<MapPose>& next_x_wall_pose_list)
    {
        // use astar path planner conected robot current pose and new start pose
        int next_pose_index = 0;
        while(1)
        {
            this->next_pose = next_x_wall_pose_list[next_pose_index];
            this->next_pose.direction = MapDirection::RIGHT;

            cv::Point start_point, end_point;
            start_point.x = this->current_pose.x;
            start_point.y = this->current_pose.y;
            end_point.x = this->next_pose.x;
            end_point.y = this->next_pose.y;

            // added by wangjiajia
            auto result = generator_.findPath({ start_point.x, start_point.y }, { end_point.x, end_point.y });
            double path_size = result.size();
            // end

            if(path_size > 0)
            {
                // add astar path to ccpp path
                for(int i=0; i<result.size(); i++)
                {
                    MapPose pose;
                    pose.x = result[i].x;
                    pose.y = result[i].y;
                    this->ccpp_path.push_back(pose);
                }

                // update next pose to current pose
                this->current_pose = this->next_pose;

                break;
            }
            else
            {
                //std::cout<<"astar planner plan fail"<<std::endl;
                next_pose_index = next_pose_index + 2;
                if(next_pose_index >= next_x_wall_pose_list.size())
                {
                    std::cout<<"connected left next cols fail"<<std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    void CCPP::getContourWallPose(const cv::Mat& ccpp_map,
                                  const MapPose& start_pose,
                                  std::vector<std::vector<MapPose>>& wall_pose_list)
    {

        int y_step = 2;
        int x_step = 2;
        for(int y=0; y<ccpp_map.rows; y=y+y_step)
        {
            std::vector<MapPose> x_wall_pose_list;
            for(int x=0; x<(ccpp_map.cols-1); x++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y,x+1) == 255)
                {
                    MapPose x_pose;
                    x_pose.x = x+1;
                    x_pose.y = y;
                    x_wall_pose_list.push_back(x_pose);
                }
                else if(ccpp_map.at<uchar>(y,x) == 255 &&
                        ccpp_map.at<uchar>(y,x+1) == 0)
                {
                    MapPose x_pose;
                    x_pose.x = x;
                    x_pose.y = y;
                    x_wall_pose_list.push_back(x_pose);
                }
            }

            if(!x_wall_pose_list.empty())
            {
                wall_pose_list.push_back(x_wall_pose_list);
            }
        }

        // y direction wall pose
        for(int x= 0 ; x<ccpp_map.cols; x=x+x_step)
        {
            std::vector<MapPose> y_wall_pose_list;
            for(int y=0; y<(ccpp_map.rows-1); y++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y+1,x) == 255)
                {
                    MapPose y_pose;
                    y_pose.x = x;
                    y_pose.y = y+1;
                    y_wall_pose_list.push_back(y_pose);
                }
                else if(ccpp_map.at<uchar>(y,x) == 255 &&
                        ccpp_map.at<uchar>(y+1,x) == 0)
                {
                    MapPose y_pose;
                    y_pose.x = x;
                    y_pose.y = y;
                    y_wall_pose_list.push_back(y_pose);
                }
            }

            if(!y_wall_pose_list.empty())
            {
                wall_pose_list.push_back(y_wall_pose_list);
            }
        }

    }

    void CCPP::getCoverageWallPose(const cv::Mat& ccpp_map,
                                   const MapPose& start_pose,
                                                std::vector<std::vector<MapPose>>& wall_pose_list,
                                                std::vector<std::vector<MapPose>>& turn_wall_pose_list)
    {
        int y_step = int(this->coverage_interval_distance / this->coverage_resolution / 2.0);
        for(int y=start_pose.y; y<ccpp_map.rows; y=y+y_step)
        {
            std::vector<MapPose> x_wall_pose_list;
            for(int x=0; x<(ccpp_map.cols-1); x++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y,x+1) == 255)
                {
                    MapPose x_pose;
                    x_pose.x = x+1;
                    x_pose.y = y;
                    x_wall_pose_list.push_back(x_pose);
                }
                else if(ccpp_map.at<uchar>(y,x) == 255 &&
                        ccpp_map.at<uchar>(y,x+1) == 0)
                {
                    MapPose x_pose;
                    x_pose.x = x;
                    x_pose.y = y;
                    x_wall_pose_list.push_back(x_pose);
                }
            }

            if(!x_wall_pose_list.empty())
            {
                if(!((y-start_pose.y)%(y_step*2))){
                    wall_pose_list.push_back(x_wall_pose_list);
                }
                else{
                    turn_wall_pose_list.push_back(x_wall_pose_list);
                }
            }
        }

        while(turn_wall_pose_list.size() >= wall_pose_list.size())
        {
            turn_wall_pose_list.pop_back();
        }
    }

    CoverageDirection CCPP::getCoverageDirection(const cv::Mat& ccpp_map)
    {
        int x_max=0, x_min=0, y_max=0, y_min=0;

        // get y_min
        for(int y=0; y<ccpp_map.rows; y++)
        {
            bool break_flag = false;
            for(int x=0; x<(ccpp_map.cols-1); x++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y,x+1) == 255)
                {
                    y_min = y;
                    break_flag = true;
                    break;
                }
            }
            if(break_flag){
                break;
            }
        }

        // get y_max
        for(int y=ccpp_map.rows-1; y>=0; y--)
        {
            bool break_flag = false;
            for(int x=0; x<(ccpp_map.cols-1); x++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y,x+1) == 255)
                {
                    y_max = y;
                    break_flag = true;
                    break;
                }
            }
            if(break_flag){
                break;
            }
        }

        // get x_min
        for(int x=0; x<ccpp_map.cols; x++)
        {
            bool break_flag = false;
            for(int y=0; y<(ccpp_map.rows-1); y++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y+1,x) == 255)
                {
                    x_min = x;
                    break_flag = true;
                    break;
                }
            }
            if(break_flag){
                break;
            }
        }

        // get x_max
        for(int x=ccpp_map.cols-1; x>=0; x--)
        {
            bool break_flag = false;
            for(int y=0; y<(ccpp_map.rows-1); y++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y+1,x) == 255)
                {
                    x_max = x;
                    break_flag = true;
                    break;
                }
            }
            if(break_flag){
                break;
            }
        }

        // calculate coverage direction
        if(abs(x_max-x_min) > abs(y_max-y_min))
        {
            return CoverageDirection::HORIZONTAL;
        }
        else
        {
            return CoverageDirection::VERTICAL;
        }
    }

    void CCPP::getCoverageMap(const cv::Mat& in_map,
                              const std::vector<MapPose>& edge_path,
                              cv::Mat& out_map)
    {
        /// set ccpp edge outside area to 0
        cv::Mat img_mask(in_map.rows, in_map.cols, CV_8UC1, cv::Scalar(255));

        cv::line(img_mask,
                 cv::Point(edge_path[0].x, edge_path[0].y),
                 cv::Point(edge_path.back().x, edge_path.back().y),
                 cv::Scalar(0), 2, 8, 0);

        for(int i=0; i<(edge_path.size()-1); i++)
        {
            cv::line(img_mask,
                     cv::Point(edge_path[i].x, edge_path[i].y),
                     cv::Point(edge_path[i+1].x, edge_path[i+1].y),
                     cv::Scalar(0), 2, 8, 0);
        }

        cv::floodFill(img_mask, cv::Point(0,0), cv::Scalar(0));

        in_map.copyTo(out_map, img_mask);

        /// get ccpp map

        // get edge area max and min value
        int edge_x_max=INT_MIN,edge_x_min=INT_MAX,edge_y_max=INT_MIN,edge_y_min=INT_MAX;
        for(int i=0; i<edge_path.size();i++)
        {
            edge_x_max = std::max(edge_x_max,edge_path[i].x);
            edge_x_min = std::min(edge_x_min,edge_path[i].x);
            edge_y_max = std::max(edge_y_max,edge_path[i].y);
            edge_y_min = std::min(edge_y_min,edge_path[i].y);
        }

        edge_x_max += this->coverage_interval_distance / this->coverage_resolution;
        edge_x_min -= this->coverage_interval_distance / this->coverage_resolution;
        edge_y_max += this->coverage_interval_distance / this->coverage_resolution;
        edge_y_min -= this->coverage_interval_distance / this->coverage_resolution;

        if(edge_x_max > in_map.cols){
            edge_x_max = in_map.cols;
        }
        if(edge_x_min < 0){
            edge_x_min = 0;
        }
        if(edge_y_max > in_map.rows){
            edge_y_max = in_map.rows;
        }
        if(edge_y_min < 0){
            edge_y_min = 0;
        }

        out_map = out_map(cv::Rect(edge_x_min, edge_y_min,
                                   abs(edge_x_max - edge_x_min),abs(edge_y_max - edge_y_min)));

        this->ccpp_map_origin_x = edge_x_min;
        this->ccpp_map_origin_y = edge_y_min;

    }

    bool CCPP::findStartPose(const cv::Mat& ccpp_map, MapPose& start_pose)
    {
        for(int y=0; y<ccpp_map.rows; y++)
        {
            for(int x=0; x<(ccpp_map.cols-1); x++)
            {
                if(ccpp_map.at<uchar>(y,x) == 0 &&
                   ccpp_map.at<uchar>(y,x+1) == 255)
                {
                    start_pose.x = x+1;
                    start_pose.y = y;
                    return true;
                }
            }
        }

        return false;
    }

    void CCPP::downsamplePath(const std::vector<MapPose>& original_path,
                                           std::vector<MapPose>& downsampled_path,
                                           MapPose& original_path_start_pose,
                                           const double path_eps)
    {
        // downsample path
        for(size_t path_point=0; path_point<original_path.size(); ++path_point)
        {
            cv::Point norm_point_a, norm_point_b;
            norm_point_a.x = original_path_start_pose.x;
            norm_point_a.y = original_path_start_pose.y;
            norm_point_b.x = original_path[path_point].x;
            norm_point_b.y = original_path[path_point].y;
            if(cv::norm(norm_point_a - norm_point_b) >= path_eps)
            {
                downsampled_path.push_back(original_path[path_point]);
                original_path_start_pose = original_path[path_point];
            }
        }
        // add last element
        if (original_path.size() > 0)
        {
            downsampled_path.push_back(original_path.back());
            original_path_start_pose = original_path.back();
        }
    }
}
