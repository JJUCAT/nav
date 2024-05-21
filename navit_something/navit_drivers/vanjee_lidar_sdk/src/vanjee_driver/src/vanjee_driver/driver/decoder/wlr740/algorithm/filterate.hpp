/*
    WLR740 补点算法
*/

#include <mutex>
#include <unordered_map>


#include <vanjee_driver/driver/decoder/wlr740/decoder/one_point_info_manage.hpp>
#include <vanjee_driver/driver/decoder/decoder.hpp>

namespace vanjee
{
    namespace lidar
    {
        class Filterate
        {
        private:
            static Filterate* _;
            static std::mutex ThisMutex;

            Trigon trigon_;

            bool AlgorithmEnb = false;
            bool BlankPointCompenEnb = false;

            bool OutlierEnb = false;
            int OutlierNum = 4;
            double OutlierRadius = 1.5;

            bool CloseRangeOutliersEnb = false;
            int CloseRangeOutliersNum = 10;
            double CloseRangeOutliersRadius = 1.5;

            int StartAngle = 0;
            unsigned int FullSearchScope = 5000; //完全搜索范围  此范围内的点 不进行跳点查找

            int corrspond[8][6] = {
                {1,9 ,17,25,33,41},
                {5,13,21,29,37,45},
                {2,10,18,26,34,42},
                {6,14,22,30,38,46},
                {3,11,19,27,35,43},
                {7,15,23,31,39,47},
                {4,12,20,28,36,44},
                {8,16,24,32,40,48}};
            int LineGroup1[48];
            std::vector<std::vector<int>> corrspondLine;
            int HResolution = 1; // 分辨率
        public:

            Filterate();
            static Filterate* GetInstance();

            std::vector<int> GetCorrspondLine(int Passage);

            void SetHResolution(int val);

            void SetStartAngle(int val) { StartAngle = val; }    // 分辨率为0.1 时配置为 1  当分辨率为0.2时 收到的为奇数角配置为1 偶数角配置为0，
            inline int GetStartAngle() { return StartAngle; }
            unsigned int GetFullSearchScope() const& {return FullSearchScope; }


            void SetAlgorithmEnb(bool val) { AlgorithmEnb = val; }

            void SetBlankPointCompenEnb(bool val) { BlankPointCompenEnb = val; }

            void SetOutlierEnb(bool val) { OutlierEnb = val; }

            void SetCloseRangeOutliersEnb(bool val) { CloseRangeOutliersEnb = val; }

            /// @brief 点云算法处理 注意：和回波模式要进行对应否则可能会造成部分点云不显示
            /// @param points  当前一圈的点云
            /// @param Prepoints 上一圈的点云
            /// @param mode 1-只处理最强一重 2-只处理最后一重 3-最强最后都处理 
            void filter(OnePointInfoManage &points,OnePointInfoManage &Prepoints,int mode = 3);
          
            void BlankPointCompenV5(OnePointInfoManage &points);

            void BlankPointCompenV5Process(OnePointInfo& point, OnePointInfoManage& points);

            void _40BlankPointCompenProcessV5(OnePointInfoManage &points);

            void _40BlankPointCompenProcessV6(OnePointInfoManage &points);

            void ExpansionProcess(OnePointInfoManage& points);

            inline void FilterNoiseV2(OnePointInfoManage &points,OnePointInfoManage &Prepoints);

            inline void FilterNoiseProcess(int init, OnePointInfo &point,OnePointInfo &Prepoint,OnePointInfoManage &points);

            inline void CloseRangeOutliers(OnePointInfoManage &points,OnePointInfoManage &Prepoints);

            inline void CloseRangeOutliersProcess(int init, OnePointInfo& point,OnePointInfoManage &points);

            /// @brief 点云算法处理 注意：和回波模式要进行对应否则可能会造成部分点云不显示
            /// @param points  当前一圈的点云
            /// @param Prepoints 上一圈的点云
            /// @param mode 1-只处理最强一重 2-只处理最后一重 3-最强最后都处理 
            inline void publicIterate(OnePointInfoManage &points,OnePointInfoManage &Prepoints,int mode);

        private:
            void DoublePointAvg(OnePointInfo& src, OnePointInfo& p1,OnePointInfo& p2);
            inline void Angle38Check(int& angle1, int& angle2, int cuLine, int checkLine);
        };
        Filterate* Filterate::_;
        std::mutex Filterate::ThisMutex;
        Filterate::Filterate()
        {
            for(int i = 0; i < 48;i++)
            {
                LineGroup1[i] = i;   // LD1-LD16 对应 1-48线
                corrspondLine.push_back(GetCorrspondLine(i+1));
            }
        }


        Filterate* Filterate::GetInstance()
        {
            //  这里使用了两个 if 判断语句的技术称为双检锁；好处是，只有判断指针为空的时候才加锁，
            //  避免每次调用 GetInstance的方法都加锁，锁的开销毕竟还是有点大的。
            if (_ == nullptr)
            {
                std::unique_lock<std::mutex> lock(ThisMutex); // 加锁
                if (_ == nullptr)
                {
                    auto temp = new (std::nothrow) Filterate();
                    _ = temp;
                }
            }

            return _;
        }

        void Filterate::SetHResolution(int val)
        {
            HResolution = val;
        }

        std::vector<int> Filterate::GetCorrspondLine(int Passage)
        {
            std::vector<int> res;
            for(int row = 0;row < 8;row++)
            {
                for(int col = 0;col < 6;col++)
                {
                    if(Passage == corrspond[row][col])
                    {
                        for(int col1 = 0;col1 < 6;col1++)
                        {
                            int RotatingMirror = 0,l_LineNo = 0;

                            l_LineNo = (corrspond[row][col1] - 1)*3+RotatingMirror + 2;
                            res.push_back(l_LineNo);
                            RotatingMirror++;

                            l_LineNo = (corrspond[row][col1] - 1)*3+RotatingMirror + 0;
                            res.push_back(l_LineNo);
                            RotatingMirror++;

                            l_LineNo = (corrspond[row][col1] - 1)*3+RotatingMirror + 1;
                            res.push_back(l_LineNo);

                        }
                        return res;
                    }
                }
            }
            return res;
        }



        inline void Filterate::filter(OnePointInfoManage &points,OnePointInfoManage &Prepoints, int mode)
        {

            if(!AlgorithmEnb)
                return;

            //auto start = std::chrono::steady_clock::now();

            // if(BlankPointCompenEnb)
            // {
            //     _40BlankPointCompenProcessV6(points);
            //     BlankPointCompenV5(points);
            // }
            // if(OutlierEnb)
            // {
            //     FilterNoiseV2(points, Prepoints); // 半径滤波 && 近距离0.2弱光分辨率处理
            // }
            
            // if(CloseRangeOutliersEnb) // 近距离0.2弱光分辨率处理
            // {
            //     CloseRangeOutliers(points, Prepoints);
            // }

            // 膨胀算法 默认关闭
            //ExpansionProcess(points); 

            publicIterate(points, Prepoints, mode);

            // auto end = std::chrono::steady_clock::now();
            // std::chrono::duration<double, std::micro> elapsed1 = end - start;

            // std::stringstream logc;
            // logc << "总耗时：" << elapsed1.count()
            // << " 微秒";
            // std::cout << logc.str();
        }

        void Filterate::publicIterate(OnePointInfoManage &points,OnePointInfoManage &Prepoints,int mode)
        {
            if(mode == 1 || mode == 3)
            {
                for(int line = 1; line <= 144;line++)
                {
                    for(int angle = StartAngle; angle < 1200;angle+=HResolution)
                    {
                        OnePointInfo& point = points.GetData(1,line,angle);
                        OnePointInfo& Prepoint = Prepoints.GetData(1,line,angle);

                        if(line == 144 && angle == 1199)
                        {
                            FilterNoiseProcess(1, point, Prepoint, points);
                            CloseRangeOutliersProcess(1, point, points);
                        }
                        else
                        {
                            FilterNoiseProcess(0, point, Prepoint, points);
                            CloseRangeOutliersProcess(0,point,points);
                            BlankPointCompenV5Process(point,points);
                        }
                    }
                }
            }
            if(mode == 2 || mode == 3)
            {
                for(int line = 1; line <= 144;line++)
                {
                    for(int angle = StartAngle; angle < 1200;angle+=HResolution)
                    {
                        OnePointInfo& point = points.GetData(0,line,angle);
                        OnePointInfo& Prepoint = Prepoints.GetData(0,line,angle);

                        if(line == 144 && angle == 1199)
                        {
                            FilterNoiseProcess(1, point, Prepoint, points);
                            CloseRangeOutliersProcess(1, point, points);
                        }
                        else
                        {
                            FilterNoiseProcess(0, point, Prepoint, points);
                            CloseRangeOutliersProcess(0,point,points);
                            BlankPointCompenV5Process(point,points);
                        }
                    }
                }
            }         
        }


        inline void Filterate::BlankPointCompenV5(OnePointInfoManage &points)
        {
            static OnePointInfo leftPoint; // 左边点
            static OnePointInfo rightPoint; // 右边点
            static int PointsZeroFlag = 0;
            static int PointsZeroCn = 0;
            static int Config_ContinuousZeroCn = 5; // 连续5个点为 0 则不进行补点，--> 因此 只支持 4个点为 0 值 才能补
            static int Config_IntervalDistance = 200; // 20cm
            if(HResolution == 1)
            {
                Config_ContinuousZeroCn = 5;
            }
            else
            {
                Config_ContinuousZeroCn = 10;
            }

            for(int line = 1; line <= 144;line++)
            {
                PointsZeroFlag = 0;
                PointsZeroCn = 0;
                for(int angle = 1; angle < 1200;angle+=HResolution)
                {
                    OnePointInfo& point = points.GetData(1,line,angle);

                    // 如果当前的点的距离不为0
                    if( point.distance !=0 )
                    {
                        // 如果之前有点为 0 则更新左边点
                        if(PointsZeroFlag == 1)
                        {
                            rightPoint = point;

                            // 判断 和 左点间隔 多少个 点
                            int intervalIndex = rightPoint.index - leftPoint.index;
                            int intervalDis = std::abs((int)rightPoint.distance - (int)leftPoint.distance);
                            int ZeroPointAngle = leftPoint.HAngle + HResolution;

                            if( (intervalIndex > 0) &&
                                (intervalIndex < Config_ContinuousZeroCn) &&  // 防止 有丢帧 因此 需要判断 左右点的间隔 距离
                                (intervalDis <= Config_IntervalDistance))
                            {
                                int AvgDis = (leftPoint.distance + rightPoint.distance) /2;
                                int AvgPW = (leftPoint.LowPulseWidth + rightPoint.LowPulseWidth) /2;
                                int intensity = (leftPoint.intensity + rightPoint.intensity) /2;
                                // float r = (leftPoint.r + rightPoint.r) / 2;
                                // float g = (leftPoint.g + rightPoint.g) / 2;
                                // float b = (leftPoint.b + rightPoint.b) / 2;

                                //while(--intervalIndex)
                                for(int i = 0; i < intervalIndex; i+= HResolution)
                                {
                                    // 注意 ：用 OnePointInfo& src 获取值 GetData数据line为0 时 不要去改变 src数据 因为返回是全局变量
                                    OnePointInfo& src = points.GetData(1,point.line,ZeroPointAngle);
                                    ZeroPointAngle += HResolution;
                                    // 这里 有个 问题 丢帧 导致丢数据时 vazimuth azimuth 是为 0 因此 无法正确补点 ，硬要补 也不能不能补 无非就是要计算 水平角即可
                                    if(src.line == 0)
                                        continue;
                                    double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                                    double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                                    double SinRidThousandth = SIN((int)src.vazimuth) *0.001;

                                    src.distance = AvgDis;
                                    src.LowPulseWidth = AvgPW;
                                    src.intensity = intensity;
                                    src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                                    src.x = src.distance * CosRidCos;
                                    src.z =  src.distance * SinRidThousandth;//不补偿
                                    src.BlankPointCompen = 1;
                                    //if(!BlankPointCompenHighlight)
                                    //{
                                    //    src.r = r;
                                    //    src.g = g;
                                    //    src.b = b;
                                    //}
                                    // else
                                    // {
                                    //     src.r = 0;
                                    //     src.g = 255;
                                    //     src.b = 0;
                                    // }
                                }
                            }
                            else if((intervalIndex > 0) &&
                                    (intervalIndex < 30))
                            {

                                for(int i = 0; i < intervalIndex; i+= HResolution)
                                {
                                    // 注意 ：用 OnePointInfo& src 获取值 GetData数据line为0 时 不要去改变 src数据 因为返回是全局变量
                                    OnePointInfo& src = points.GetData(1,point.line,ZeroPointAngle);

                                    int upDiff = 1;
                                    int downDiff = 1;
                                    if(point.StrongWeak == 2)
                                    {
                                        // 弱光的上下点的获取需要单独区分
                                        if(src.line == 23)
                                        {
                                            upDiff = 2;
                                            downDiff = 3;
                                        }
                                        else if(src.line == 25)
                                        {
                                            upDiff = 3;
                                            downDiff = 2;
                                        }
                                        else if(src.line == 46)
                                        {
                                            upDiff = 5;
                                            downDiff = 3;
                                        }
                                        else if(src.line == 51)
                                        {
                                            upDiff = 3;
                                            downDiff = 5;
                                        }
                                        else if(src.line == 72)
                                        {
                                            upDiff = 2;
                                            downDiff = 3;
                                        }
                                        else if(src.line == 74)
                                        {
                                            upDiff = 3;
                                            downDiff = 2;
                                        }
                                        else if(src.line == 95)
                                        {
                                            upDiff = 2;
                                            downDiff = 3;
                                        }
                                        else if(src.line == 118)
                                        {
                                            upDiff = 5;
                                            downDiff = 3;
                                        }
                                        else if(src.line == 123)
                                        {
                                            upDiff = 3;
                                            downDiff = 5;
                                        }
                                        else
                                        {
                                            upDiff = 3;
                                            downDiff = 3;
                                        }
                                    }

                                    if( (src.line!=0) &&
                                        ((src.line + upDiff) >= 1 && (src.line + upDiff) <= 144) &&
                                        ((src.line - downDiff) >= 1 && (src.line - downDiff) <= 144))
                                    {
                                        OnePointInfo& upPoint = points.GetData(1,src.line + upDiff,ZeroPointAngle);
                                        OnePointInfo& DownPoint = points.GetData(1,src.line - downDiff,ZeroPointAngle);

                                        if((upPoint.distance == 0) || (DownPoint.distance == 0))
                                            continue;

                                        intervalDis = std::abs((int)upPoint.distance - (int)DownPoint.distance);
                                        if(intervalDis <= Config_IntervalDistance)
                                        {
                                            int AvgDis = (upPoint.distance + DownPoint.distance) /2;
                                            int AvgPW = (upPoint.LowPulseWidth + DownPoint.LowPulseWidth) /2;
                                            int intensity = (upPoint.intensity + DownPoint.intensity) /2;
                                            // float r = (upPoint.r + DownPoint.r) / 2;
                                            // float g = (upPoint.g + DownPoint.g) / 2;
                                            // float b = (upPoint.b + DownPoint.b) / 2;

                                            double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                                            double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                                            double SinRidThousandth = SIN((int)src.vazimuth) *0.001;

                                            src.distance = AvgDis;
                                            src.LowPulseWidth = AvgPW;
                                            src.intensity = intensity;
                                            src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                                            src.x = src.distance * CosRidCos;
                                            src.z =  src.distance * SinRidThousandth;//不补偿
                                            src.BlankPointCompen = 1;
                                            //if(!BlankPointCompenHighlight)
                                            //{
                                            //    src.r = r;
                                            //    src.g = g;
                                            //    src.b = b;
                                            //}
                                            //else
                                            // {
                                            //     src.r = 0;
                                            //     src.g = 255;
                                            //     src.b = 0;
                                            // }

                                        }
                                    }
                                    ZeroPointAngle += HResolution;

                                }

                                // 查看 上点和下点

                            }

                            leftPoint = point; // 更新右边点
                            PointsZeroFlag = 0; //补点完成 标志位 置 0
                            PointsZeroCn = 0;
                        }
                        else
                        {
                            leftPoint = point; // 更新右边点
                            PointsZeroFlag = 0;
                            PointsZeroCn = 0;
                        }
                    }
                    else
                    {
                        // 如果当前的点的 距离为 0

                        // 检查他的 上点 和下点  上下点 都有值 则 补点


                        PointsZeroFlag = 1; // 将标志位 置 1

                        if(PointsZeroFlag == 1)
                        {
                            PointsZeroCn++;

                            // 如果超过了 配置 补点最大值 则  不能补点
                            if(PointsZeroCn >= 30)
                            {
                                PointsZeroFlag = 0; // 清空 标志位
                                PointsZeroCn = 0;   // 清空 连续 0点次数
                            }
                        }
                    }


                }
            }
        }

        inline void Filterate::BlankPointCompenV5Process(OnePointInfo& point, OnePointInfoManage& points)
        {
            static OnePointInfo leftPoint; // 左边点
            static OnePointInfo rightPoint; // 右边点
            static int PointsZeroFlag = 0;
            static int PointsZeroCn = 0;
            static int Config_ContinuousZeroCn = 5; // 连续5个点为 0 则不进行补点，--> 因此 只支持 4个点为 0 值 才能补
            static int Config_IntervalDistance = 300; // 20cm

            static int PreLine = 0;

            if(!BlankPointCompenEnb)
                return;

            if(HResolution == 1)
            {
                Config_ContinuousZeroCn = 5;
            }
            else
            {
                Config_ContinuousZeroCn = 10;
            }

            if(point.line != PreLine)
            {
                PointsZeroFlag = 0;
                PointsZeroCn = 0;
                PreLine = point.line;
            }

            // 如果当前的点的距离不为0
            if( point.distance !=0 )
            {
                // 如果之前有点为 0 则更新左边点
                if(PointsZeroFlag == 1)
                {
                    rightPoint = point;

                    // 判断 和 左点间隔 多少个 点
                    int intervalIndex = rightPoint.index - leftPoint.index;
                    int intervalDis = std::abs((int)rightPoint.distance - (int)leftPoint.distance);
                    int ZeroPointAngle = leftPoint.HAngle + HResolution;

                    if( (intervalIndex > 0) &&
                        (intervalIndex <= Config_ContinuousZeroCn) &&  // 防止 有丢帧 因此 需要判断 左右点的间隔 距离
                        (intervalDis <= Config_IntervalDistance))
                    {
                        int AvgDis = (leftPoint.distance + rightPoint.distance) /2;
                        int AvgPW = (leftPoint.LowPulseWidth + rightPoint.LowPulseWidth) /2;
                        int intensity = (leftPoint.intensity + rightPoint.intensity) /2;
                        float r = (leftPoint.r + rightPoint.r) / 2;
                        float g = (leftPoint.g + rightPoint.g) / 2;
                        float b = (leftPoint.b + rightPoint.b) / 2;

                        //while(--intervalIndex)
                        for(int i = 0; i < intervalIndex-1; i+= HResolution)
                        {
                            // 注意 ：用 OnePointInfo& src 获取值 GetData数据line为0 时 不要去改变 src数据 因为返回是全局变量
                            OnePointInfo& src = points.GetData(point.EchoType,point.line,ZeroPointAngle);
                            ZeroPointAngle += HResolution;
                            // 这里 有个 问题 丢帧 导致丢数据时 vazimuth azimuth 是为 0 因此 无法正确补点 ，硬要补 也不能不能补 无非就是要计算 水平角即可
                            if(src.line == 0)
                                continue;
                            double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                            double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                            double SinRidThousandth = SIN((int)src.vazimuth) *0.001;

                            src.distance = AvgDis;
                            src.LowPulseWidth = AvgPW;
                            src.intensity = intensity;
                            src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                            src.x = src.distance * CosRidCos;
                            src.z =  src.distance * SinRidThousandth;//不补偿
                            src.BlankPointCompen = 1;
                            src.NonFilterable = 1; // 补点默认 不当作离群点
                            src.CloseRangeOutliers = 1; //

                        }
                    }
                    else if((intervalIndex > 0) &&
                            (intervalIndex < 30))
                    {

                        for(int i = 0; i < intervalIndex; i+= HResolution)
                        {
                            // 注意 ：用 OnePointInfo& src 获取值 GetData数据line为0 时 不要去改变 src数据 因为返回是全局变量
                            OnePointInfo& src = points.GetData(point.EchoType,point.line,ZeroPointAngle);

                            int upDiff = 1;
                            int downDiff = 1;
                            if(point.StrongWeak == 2)
                            {
                                // 弱光的上下点的获取需要单独区分
                                if(src.line == 23)
                                {
                                    upDiff = 2;
                                    downDiff = 3;
                                }
                                else if(src.line == 25)
                                {
                                    upDiff = 3;
                                    downDiff = 2;
                                }
                                else if(src.line == 46)
                                {
                                    upDiff = 5;
                                    downDiff = 3;
                                }
                                else if(src.line == 51)
                                {
                                    upDiff = 3;
                                    downDiff = 5;
                                }
                                else if(src.line == 72)
                                {
                                    upDiff = 2;
                                    downDiff = 3;
                                }
                                else if(src.line == 74)
                                {
                                    upDiff = 3;
                                    downDiff = 2;
                                }
                                else if(src.line == 95)
                                {
                                    upDiff = 2;
                                    downDiff = 3;
                                }
                                else if(src.line == 118)
                                {
                                    upDiff = 5;
                                    downDiff = 3;
                                }
                                else if(src.line == 123)
                                {
                                    upDiff = 3;
                                    downDiff = 5;
                                }
                                else
                                {
                                    upDiff = 3;
                                    downDiff = 3;
                                }
                            }

                            if( (src.line!=0) &&
                                ((src.line + upDiff) >= 1 && (src.line + upDiff) <= 144) &&
                                ((src.line - downDiff) >= 1 && (src.line - downDiff) <= 144))
                            {
                                OnePointInfo& upPoint = points.GetData(point.EchoType,src.line + upDiff,ZeroPointAngle);
                                OnePointInfo& DownPoint = points.GetData(point.EchoType,src.line - downDiff,ZeroPointAngle);

                                if((upPoint.distance == 0) || (DownPoint.distance == 0))
                                    continue;

                                intervalDis = std::abs((int)upPoint.distance - (int)DownPoint.distance);
                                if(intervalDis <= Config_IntervalDistance)
                                {
                                    int AvgDis = (upPoint.distance + DownPoint.distance) /2;
                                    int AvgPW = (upPoint.LowPulseWidth + DownPoint.LowPulseWidth) /2;
                                    int intensity = (upPoint.intensity + DownPoint.intensity) /2;
                                    float r = (upPoint.r + DownPoint.r) / 2;
                                    float g = (upPoint.g + DownPoint.g) / 2;
                                    float b = (upPoint.b + DownPoint.b) / 2;

                                    double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                                    double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                                    double SinRidThousandth = SIN((int)src.vazimuth) *0.001;

                                    src.distance = AvgDis;
                                    src.LowPulseWidth = AvgPW;
                                    src.intensity = intensity;
                                    src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                                    src.x = src.distance * CosRidCos;
                                    src.z =  src.distance * SinRidThousandth;//不补偿
                                    src.BlankPointCompen = 1;
                                    src.NonFilterable = 1; // 补点默认不作为离群点
                                    src.CloseRangeOutliers = 1; //

                                }
                            }
                            ZeroPointAngle += HResolution;

                        }

                        // 查看 上点和下点

                    }

                    leftPoint = point; // 更新右边点
                    PointsZeroFlag = 0; //补点完成 标志位 置 0
                    PointsZeroCn = 0;
                }
                else
                {
                    leftPoint = point; // 更新左边点
                    PointsZeroFlag = 0;
                    PointsZeroCn = 0;
                }
            }
            else
            {
                // 如果当前的点的 距离为 0

                // 检查他的 上点 和下点  上下点 都有值 则 补点


                PointsZeroFlag = 1; // 将标志位 置 1

                if(PointsZeroFlag == 1)
                {
                    PointsZeroCn++;

                    // 如果超过了 配置 补点最大值 则  不能补点
                    if(PointsZeroCn >= 30)
                    {
                        PointsZeroFlag = 0; // 清空 标志位
                        PointsZeroCn = 0;   // 清空 连续 0点次数
                    }
                }
            }

        }


        inline void Filterate::_40BlankPointCompenProcessV5(OnePointInfoManage &points)
        {

            OnePointInfo LeftHighPoint; // 从入栈的顺序来看 先入栈的为左 后入栈 的为右
            OnePointInfo RigthHighPoint;
            int continuous = 2;
            for(int LineNo = 1; LineNo < 145;LineNo++) // 144 线 * 1600水平角度
            {
                std::vector<OnePointInfo> repaired;
                for(unsigned int Angle = 0; Angle < 1200;Angle++)
                {
                    OnePointInfo& tem = points.GetData(1,LineNo,Angle);

                    // 130米后的鬼影
                    if(tem.distance > 130000)
                    {
                        if(tem.LowPulseWidth > 19000) // 脉宽大于 19ns 检查同时发光的LD
                        {
                            std::vector<int> Lines = corrspondLine[tem.Channal-1];
                            //int MirrorNo = tem.Mirror;
                            int HAngle = tem.HAngle;
                            for(int idx = 0;idx < Lines.size();idx++)
                            {
                                if(tem.line != Lines[idx] && (Lines[idx] > 85)) // 自己就不用检测了
                                {
                                    long Key = points.GetIndex(1,Lines[idx],HAngle);
                                    if(Key == -1)
                                        continue;
                                    auto& p = points.GetDataIt(Key);

                                    if(p.LowPulseWidth < 9000 && std::abs((int)p.distance - (int)tem.distance) < 300)
                                    {
                                        //if(GhostHighlight)
                                        //{
                                        //    p.r = GhostHighlightColor.redF();
                                        //    p.g = GhostHighlightColor.greenF();
                                        //    p.b = GhostHighlightColor.blueF();
                                        //}
                                        //else
                                        {
                                            p.distance = 0;
                                            p.x = 0;
                                            p.y = 0;
                                            p.z = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }//end 13米鬼影



                    if(tem.x == 0 && tem.y == 0 && tem.z == 0)
                    {
                        continue;
                    }

                    if(tem.LowPulseWidth > 25000)
                    {
                        if(continuous == 1)
                        {
                            RigthHighPoint = tem;

                            for(int i = 0; i < repaired.size(); i++)
                            {

                                OnePointInfo p1;
                                OnePointInfo p2;
                                bool OK = false;
                                if((std::abs((int)RigthHighPoint.distance - (int)LeftHighPoint.distance)  < 200))
                                {
                                    p1 = RigthHighPoint;
                                    p2 = LeftHighPoint;
                                    OK = true;
                                }

                                if(OK)
                                {
                                    int diff = std::abs((int)p1.distance - (int)repaired[i].distance);
                                    if(diff >= 38000 && diff <= 42000)
                                    {
                                        DoublePointAvg(repaired[i],p1,p2);
                                        repaired[i].BlankPointCompen = 1;
                                        //if(!BlankPointCompenHighlight)
                                        //{
                                            repaired[i].r = (p2.r + p1.r) / 2;
                                            repaired[i].g = (p2.g + p1.g) / 2;
                                            repaired[i].b = (p2.b + p1.b) / 2;
                                        // }
                                        // else
                                        // {
                                        //     repaired[i].r = 255;
                                        //     repaired[i].g = 0;
                                        //     repaired[i].b = 0;
                                        // }

                                        points.insert(repaired[i].index,repaired[i]);
                                    }
                                }
                            }
                        }

                        LeftHighPoint = tem;
                        continuous =0;
                        repaired.clear();
                    }
                    else if(tem.LowPulseWidth < 15000)
                    {

                        // 找到每个repaired点的上下点
                        bool UpDownflag = true;
                        //OnePointInfo UpPoint;
                        OnePointInfo DownPoint;

                        if((LineNo-1) > 0 && LineNo+1 <= 144)
                        {
                            //UpPoint = points.GetData(1,LineNo+1,Angle);
                            DownPoint = points.GetData(1,LineNo-1,Angle);

                            if( /*(UpPoint.x ==0 && UpPoint.y ==0 && UpPoint.z ==0) ||*/
                                (DownPoint.x ==0 && DownPoint.y ==0 && DownPoint.z ==0) ||
                                //                        UpPoint.LowPulseWidth < 25000 ||
                                DownPoint.LowPulseWidth < 25000)
                            {
                                UpDownflag = false;
                            }
                        }

                        if(UpDownflag == false)
                        {
                            UpDownflag = true;
                            if((LineNo-3) > 0 && LineNo+1 <= 144)
                            {
                                //UpPoint = points.GetData(1,LineNo+1,Angle);
                                DownPoint = points.GetData(1,LineNo-3,Angle);

                                if( /*(UpPoint.x ==0 && UpPoint.y ==0 && UpPoint.z ==0) ||*/
                                    (DownPoint.x ==0 && DownPoint.y ==0 && DownPoint.z ==0) ||
                                    //                        UpPoint.LowPulseWidth < 25000 ||
                                    DownPoint.LowPulseWidth < 25000)
                                {
                                    UpDownflag = false;
                                }
                            }
                        }

                        if(UpDownflag)
                        {
                            OnePointInfo p1;
                            OnePointInfo p2;
                            bool OK = false;
                            if(((UpDownflag)/*&&(std::abs(UpPoint.y - DownPoint.y)  <= 1)*/))
                            {
                                //p1 = UpPoint;
                                p2 = DownPoint;
                                OK = true;
                            }
                            if(p1.line == 0 && p2.line == 0)
                            {
                                OK = false;
                            }
                            else if(p1.line == 0)
                            {
                                p1 = p2;
                            }
                            else if(p2.line == 0)
                            {
                                p2 = p1;
                            }
                            else
                            {
                                OK = false;
                            }

                            if(OK)
                            {
                                int diff = std::abs((int)p1.distance - (int)tem.distance);
                                int TongIdx = -1;
                                for(int i = 0 ; i < repaired.size();i++)
                                {
                                    // 检测和当前点相同匹配的点
                                    if(std::abs((int)repaired[i].index - (int)tem.index) == 1 && (repaired[i].LowPulseWidth == tem.LowPulseWidth))
                                    {
                                        TongIdx = i;
                                    }
                                }
                                if(diff >= 38000 && diff <= 42000)
                                {
                                    DoublePointAvg(tem,p1,p2);
                                    if(TongIdx != -1)
                                    {
                                        repaired[TongIdx].distance = tem.distance;
                                        repaired[TongIdx].x = tem.x;
                                        repaired[TongIdx].y = tem.y;
                                        repaired[TongIdx].z = tem.z;
                                        repaired[TongIdx].BlankPointCompen = 1;
                                        //if(!BlankPointCompenHighlight)
                                        //{
                                            repaired[TongIdx].r = tem.r;
                                            repaired[TongIdx].g = tem.g;
                                            repaired[TongIdx].b = tem.b;
                                        // }
                                        // else
                                        // {
                                        //     repaired[TongIdx].r = 255;
                                        //     repaired[TongIdx].g = 0;
                                        //     repaired[TongIdx].b = 0;
                                        // }
                                        points.insert(repaired[TongIdx].index,repaired[TongIdx]);
                                        repaired.erase(repaired.begin()+ TongIdx);
                                    }

                                    tem.BlankPointCompen = 1;
                                    //if(!BlankPointCompenHighlight)
                                    //{
                                        tem.r = (p2.r + p1.r) / 2;
                                        tem.g = (p2.g + p1.g) / 2;
                                        tem.b = (p2.b + p1.b) / 2;
                                    // }
                                    // else
                                    // {
                                    //     tem.r = 255;
                                    //     tem.g = 0;
                                    //     tem.b = 0;
                                    // }

                                    points.insert(tem.index,tem);
                                    //continuous = 2;
                                    //repaired.clear();
                                    continue;
                                }
                            }

                        }


                        if(continuous == 0 || continuous == 1)
                        {
                            //-- 待检测
                            repaired.push_back(tem);
                            continuous = 1;
                        }
                        if(repaired.size() >= 6)
                        {
                            continuous = 2;
                            repaired.clear();
                        }
                    }
                }
            }
        }

        inline void Filterate::_40BlankPointCompenProcessV6(OnePointInfoManage &points)
        {
            // 1 9 17 25 33 41
            static const int CheckLineNo[] = { 1, 2, 3,
                                            25, 26, 27,
                                            49, 50, 51,
                                            73, 74, 75,
                                            97, 98, 99,
                                            121, 122, 123};
            static OnePointInfo leftPoint; // 左边点
            static OnePointInfo rightPoint; // 右边点
            static int PointsWeakFlag = 0;
            static int PointsWeakCn = 0;
            static int Config_ContinuousZeroCn = 5; // 连续5个点为 0 则不进行补点，--> 因此 只支持 4个点为 0 值 才能补
            static int Config_IntervalDistance = 200; // 20cm
            if(HResolution == 1)
            {
                Config_ContinuousZeroCn = 15;
            }
            else
            {
                Config_ContinuousZeroCn = 30;
            }

            int line = 1;
            for(int i = 0; i < 18;i++)
            {
                line = CheckLineNo[i];

                for(int angle = StartAngle; angle < 1200;angle+=HResolution)
                {
                    OnePointInfo& point = points.GetData(1,line,angle);

                    // 130米后的鬼影
                    if(point.distance > 130000)
                    {
                        if(point.LowPulseWidth > 19000) // 脉宽大于 19ns 检查同时发光的LD
                        {
                            std::vector<int> Lines = corrspondLine[point.Channal-1];
                            //int MirrorNo = tem.Mirror;
                            int HAngle = point.HAngle;
                            for(int idx = 0;idx < Lines.size();idx++)
                            {
                                if(point.line != Lines[idx] && (Lines[idx] > 85)) // 自己就不用检测了
                                {
                                    long Key = points.GetIndex(1,Lines[idx],HAngle);
                                    if(Key == -1)
                                        continue;
                                    auto& p = points.GetDataIt(Key);

                                    if(p.LowPulseWidth < 9000 && std::abs((int)p.distance - (int)point.distance) < 300)
                                    {
                                        // if(GhostHighlight)
                                        // {
                                        //     p.r = GhostHighlightColor.redF();
                                        //     p.g = GhostHighlightColor.greenF();
                                        //     p.b = GhostHighlightColor.blueF();
                                        // }
                                        // else
                                        {
                                            p.distance = 0;
                                            p.x = 0;
                                            p.y = 0;
                                            p.z = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }


                    if(point.distance == 0)
                        continue;
                    if(point.LowPulseWidth >= 25000)
                    {
                        if(PointsWeakFlag == 1) // 表示两个高反点之间存在弱反
                        {
                            rightPoint = point;
                            // 判断 和 左点间隔 多少个 点
                            int intervalIndex = rightPoint.index - leftPoint.index;
                            int intervalDis = std::abs((int)rightPoint.distance - (int)leftPoint.distance);
                            int NextPointAngle = leftPoint.HAngle + HResolution;

                            if( (intervalIndex > 0) &&
                                (intervalIndex < Config_ContinuousZeroCn) &&
                                (intervalDis < Config_IntervalDistance))
                            {
                                int AvgDis = (leftPoint.distance + rightPoint.distance) /2;
                                int AvgPW = (leftPoint.LowPulseWidth + rightPoint.LowPulseWidth) /2;
                                int intensity = (leftPoint.intensity + rightPoint.intensity) /2;
                                float r = (leftPoint.r + rightPoint.r) / 2;
                                float g = (leftPoint.g + rightPoint.g) / 2;
                                float b = (leftPoint.b + rightPoint.b) / 2;

                                for(int i = 0; i < intervalIndex; i+= HResolution)
                                {
                                    OnePointInfo& src = points.GetData(1,point.line,NextPointAngle);
                                    if(src.line == 0)
                                        continue;
                                    int flag = false;
                                    int diff = rightPoint.distance - src.old_distance;
                                    if(diff >= 38000 && diff <= 42000 && src.LowPulseWidth < 15000)
                                    {
                                        flag = true;
                                    }
                                    else
                                    {
                                        //                                // 检查下点
                                        //                                int DownOnePointLine = (int)point.line-1;
                                        //                                int DownThreePointLine = (int)point.line-3;
                                        //                                if((DownOnePointLine >= 1) && (DownOnePointLine <= 144))
                                        //                                {
                                        //                                    OnePointInfo& DownPoint = points.GetData(1,point.line - 1,NextPointAngle);
                                        //                                    if( DownPoint._40BlankPointCompen == 0 &&
                                        //                                        DownPoint.LowPulseWidth > 25000 &&
                                        //                                        DownPoint.distance >0)
                                        //                                    {
                                        //                                        flag = true;
                                        //                                    }
                                        //                                }
                                        //                                else if( (DownThreePointLine >= 1) && (DownThreePointLine <= 144))
                                        //                                {
                                        //                                    OnePointInfo& DownPoint = points.GetData(1,point.line - 3,NextPointAngle);
                                        //                                    if( DownPoint._40BlankPointCompen == 0 &&
                                        //                                        DownPoint.LowPulseWidth > 25000 &&
                                        //                                        DownPoint.distance > 0)
                                        //                                    {
                                        //                                        flag = true;
                                        //                                    }
                                        //                                }
                                    }

                                    if(flag)
                                    {
                                        double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                                        double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                                        double SinRidThousandth = SIN((int)src.vazimuth) *0.001;
                                        src.distance = AvgDis;
                                        src.LowPulseWidth = AvgPW;
                                        src.intensity = intensity;
                                        src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                                        src.x = src.distance * CosRidCos;
                                        src.z =  src.distance * SinRidThousandth;//不补偿
                                        src._40BlankPointCompen = 1;
                                        // if(!BlankPointCompenHighlight)
                                        // {
                                        //    src.r = r;
                                        //    src.g = g;
                                        //    src.b = b;
                                        // }
                                        // else
                                        // {
                                        //     src.r = 255;
                                        //     src.g = 0;
                                        //     src.b = 0;
                                        // }
                                    }

                                    NextPointAngle += HResolution;
                                }

                            }
                            leftPoint = point;
                            PointsWeakFlag = 0;
                            PointsWeakCn = 0;
                        }
                        else
                        {
                            leftPoint = point;
                            PointsWeakFlag = 0;
                            PointsWeakCn = 0;
                        }
                    }
                    else if(point.LowPulseWidth < 15000)
                    {
                        PointsWeakFlag = 1;


                        int flag = false;
                        // 检查下点
                        int DownOnePointLine = (int)point.line-1;
                        int DownThreePointLine = (int)point.line-3;
                        OnePointInfo DownPoint;
                        if((DownOnePointLine >= 1) && (DownOnePointLine <= 144))
                        {
                            int tem_angle = angle;
                            if(point.line == 97)
                                tem_angle-=38;
                            if(tem_angle < 0)
                                continue;
                            DownPoint = points.GetData(1,point.line - 1,tem_angle);
                            if( DownPoint._40BlankPointCompen == 0 &&
                                DownPoint.LowPulseWidth > 25000 &&
                                DownPoint.distance >0)
                            {
                                flag = true;
                            }
                        }
                        else if( (DownThreePointLine >= 1) && (DownThreePointLine <= 144))
                        {
                            int tem_angle = angle;
                            if(point.line == 97)
                                tem_angle-=38;
                            if(tem_angle < 0)
                                continue;
                            DownPoint = points.GetData(1,point.line - 3,tem_angle);
                            if( DownPoint._40BlankPointCompen == 0 &&
                                DownPoint.LowPulseWidth > 25000 &&
                                DownPoint.distance > 0)
                            {
                                flag = true;
                            }
                        }
                        if(flag)
                        {
                            OnePointInfo& src = point;

                            int AvgDis = DownPoint.distance;
                            int AvgPW = DownPoint.LowPulseWidth;
                            int intensity = DownPoint.intensity;
                            float r = DownPoint.r;
                            float g = DownPoint.g;
                            float b = DownPoint.b;


                            double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
                            double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
                            double SinRidThousandth = SIN((int)src.vazimuth) *0.001;
                            src.old_distance = src.distance;
                            src.distance = AvgDis;
                            src.LowPulseWidth = AvgPW;
                            src.intensity = intensity;
                            src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
                            src.x = src.distance * CosRidCos;
                            src.z =  src.distance * SinRidThousandth;//不补偿
                            src._40BlankPointCompen = 1;
                            // if(!BlankPointCompenHighlight)
                            // {
                            //     src.r = r;
                            //     src.g = g;
                            //     src.b = b;
                            // }
                            // else
                            // {
                            //     src.r = 255;
                            //     src.g = 0;
                            //     src.b = 0;
                            // }
                        }

                        //                if(PointsWeakFlag == 1)
                        //                {
                        //                    PointsWeakCn++;

                        //                    // 如果超过了 配置 补点最大值 则  不能拉点
                        //                    if(PointsWeakCn >= Config_ContinuousZeroCn)
                        //                    {
                        //                        PointsWeakCn = 0; // 清空 标志位
                        //                        PointsWeakCn = 0;   // 清空 连续 0点次数
                        //                    }
                        //                }
                    }


                }
            }
        }

        inline void Filterate::ExpansionProcess(OnePointInfoManage& points)
        {

            std::vector<OnePointInfo> tem_Points;
            for(int line = 1; line <= 144;line++)
            {
                tem_Points.clear();

                for(int angle = 1; angle < 1200;angle+=HResolution)
                {
                    OnePointInfo& point = points.GetData(1,line,angle);
                    if(point.LowPulseWidth > 29000)
                    {
                        tem_Points.push_back(point);
                    }
                }

                int size = tem_Points.size();
                if(size == 0)
                    continue;

                int PreAngle = tem_Points[0].azimuth;
                int PreHAngle = tem_Points[0].HAngle;
                int sum = 0;
                int StartAngle = 0;
                int EndAngle = 0;
                int dis = 0;
                double zDis = 0;
                for(int i = 0 ; i < size; i++)
                {
                    int diff = tem_Points[i].azimuth - PreAngle;
                    if( (diff <= 50) && (diff>=20) )
                    {
                        sum++;
                        dis += tem_Points[i].distance;
                        zDis += tem_Points[i].z;
                        tem_Points[i].NonFilterable = 1;
                        points.insert(tem_Points[i].index,tem_Points[i]);
                        if(StartAngle == 0)
                            StartAngle = tem_Points[i].HAngle;
                        EndAngle = tem_Points[i].HAngle;

                        while(++PreHAngle)
                        {
                            if(PreHAngle >= tem_Points[i].HAngle)
                            {
                                break;
                            }
                            OnePointInfo& tem_point = points.GetData(1,line,PreHAngle);
                            if(tem_point.distance == 0)
                                continue;
                            tem_point.NonFilterable =1;
                        }

                    }
                    else if(diff <= 50 && (i != size-1))
                    {
                        sum++;
                        dis += tem_Points[i].distance;
                        zDis += tem_Points[i].z;
                        if(StartAngle == 0)
                            StartAngle = tem_Points[i].HAngle;
                        EndAngle = tem_Points[i].HAngle;
                    }
                    else if(diff > 50 || (i == size-1))
                    {
                        if(sum > 25)
                        {
                            dis /= sum;
                            zDis /= sum;
                            // 检查上下4条线的交集比
                            if( (line - 4 >= 1) && (line - 4 <= 144) &&
                                (line + 4 >= 1) && (line + 4 <= 144))
                            {

                                int tem_StartAngle = StartAngle;
                                int tem_EndAngle = EndAngle;
                                // 如果 当前线是 45 ~ 46那么-4后 角度 要 -38
                                if(line >= 45 && line <= 48)
                                {
                                    if( ((tem_StartAngle - 38 >= 1) && (tem_StartAngle - 38 < 1200)) &&
                                        ((tem_EndAngle - 38) >= 1) && (tem_EndAngle - 38 < 1200))
                                    {
                                        tem_StartAngle -= 38;
                                        tem_EndAngle -= 38;
                                    }
                                    else
                                    {
                                        continue;
                                    }
                                }
                                else if(line >= 93 && line <= 96)
                                {
                                    if( ((tem_StartAngle + 38 >= 1) && (tem_StartAngle + 38 < 1200)) &&
                                        ((tem_EndAngle + 38) >= 1) && (tem_EndAngle + 38 < 1200))
                                    {
                                        tem_StartAngle += 38;
                                        tem_EndAngle += 38;
                                    }
                                    else
                                    {
                                        continue;
                                    }
                                }


                                int sum1 = 0;
                                int sum2 = 0;
                                int inspectLine = line + 4;
                                int inspectLine1 = line - 4;
                                int tem_PreAngle = points.GetData(1,inspectLine,tem_StartAngle).azimuth;
                                for(int j = tem_StartAngle; j < tem_EndAngle; j+=HResolution)
                                {
                                    OnePointInfo& tem_point = points.GetData(1,inspectLine,j);
                                    int diff1 = tem_point.azimuth - tem_PreAngle;
                                    if( (diff1 <= 50) && (diff1>=2) )
                                    {
                                        sum1++;
                                    }
                                    else if(diff1 <= 50)
                                    {
                                        sum1++;
                                    }
                                    else if(diff1 > 50)
                                    {
                                        break;
                                    }
                                    tem_PreAngle = tem_point.azimuth;
                                }
                                if(sum1 >= sum *0.7)
                                {
                                    tem_PreAngle = points.GetData(1,inspectLine1,tem_StartAngle).azimuth;
                                    for(int j = tem_StartAngle; j < tem_EndAngle; j+=HResolution)
                                    {
                                        OnePointInfo& tem_point = points.GetData(1, inspectLine1, j);
                                        int diff1 = tem_point.azimuth - tem_PreAngle;
                                        if( (diff1 <= 50) && (diff1>=2) )
                                        {
                                            sum2++;
                                        }
                                        else if(diff1 <= 50)
                                        {
                                            sum2++;
                                        }
                                        else if(diff1 > 50)
                                        {
                                            break;
                                        }
                                        tem_PreAngle = tem_point.azimuth;
                                    }
                                    if(sum2 >= sum *0.7)
                                    {
                                        // 上下 10条线
                                        int upLine = line;
                                        int DownLine = line;
                                        double tem_zDis = zDis;
                                        double NextZdis = 0;
                                        while( ++upLine <= 144)
                                        {
                                            tem_StartAngle = StartAngle - 20;
                                            tem_StartAngle = tem_StartAngle < 1? 1:tem_StartAngle;
                                            tem_EndAngle = EndAngle + 20;
                                            tem_EndAngle = tem_EndAngle > 1200 ? 1200:tem_EndAngle;

                                            if(line >= 1 && line <= 48)
                                            {
                                                if(upLine >= 1 && upLine <= 48)
                                                {

                                                }
                                                else if(upLine >= 49 && upLine <= 96)
                                                {
                                                    tem_StartAngle -= 38;
                                                    tem_EndAngle -= 38;
                                                }
                                                else if(upLine >= 97 && upLine <= 144)
                                                {

                                                }
                                            }
                                            else if(line >= 49 && line <= 96)
                                            {
                                                if(upLine >= 1 && upLine <= 48)
                                                {
                                                    tem_StartAngle += 38;
                                                    tem_EndAngle += 38;
                                                }
                                                else if(upLine >= 49 && upLine <= 96)
                                                {

                                                }
                                                else if(upLine >= 97 && upLine <= 144)
                                                {
                                                    tem_StartAngle += 38;
                                                    tem_EndAngle += 38;
                                                }
                                            }
                                            else if(line >= 97 && line <= 144)
                                            {
                                                if(upLine >= 1 && upLine <= 48)
                                                {

                                                }
                                                else if(upLine >= 49 && upLine <= 96)
                                                {
                                                    tem_StartAngle -= 38;
                                                    tem_EndAngle -= 38;
                                                }
                                                else if(upLine >= 97 && upLine <= 144)
                                                {

                                                }
                                            }
                                            if(tem_StartAngle < 1 || tem_StartAngle>=1200)
                                                break;
                                            if(tem_EndAngle < 1 || tem_EndAngle>=1200)
                                                break;

                                            int cn =0 ;
                                            for(int j = tem_StartAngle; j < tem_EndAngle; j+=HResolution)
                                            {

                                                OnePointInfo& tem_point = points.GetData(1, upLine, j);
                                                if(tem_point.distance == 0)
                                                    continue;
        //                                        if((std::abs(tem_zDis - tem_point.z) > 2) &&
        //                                            (j > tem_StartAngle+30) &&
        //                                            (j < tem_EndAngle - 30))
        //                                        {
        //                                            break;
        //                                        }

                                                if( (std::abs(dis - (int)tem_point.distance) <= 150) &&
                                                    (tem_point.LowPulseWidth < 16000) &&
                                                    (tem_point.NonFilterable == 0))
                                                {
                                                    NextZdis += tem_point.z;
                                                    cn++;
                                                    tem_point.Expansion = 1;

                                                    // if(ExpansionHighlinght)
                                                    // {
                                                    //     tem_point.r = 255;
                                                    //     tem_point.g = 0;
                                                    //     tem_point.b = 255;
                                                    // }
                                                    //else
                                                    {
                                                        tem_point.x = 0;
                                                        tem_point.y = 0;
                                                        tem_point.z = 0;
                                                        tem_point.distance = 0;
                                                    }
                                                }
                                            }

                                            if(cn > 0)
                                            {
                                                NextZdis /= cn;
                                                tem_zDis = NextZdis;
                                            }


                                            if(line+10 == upLine)
                                                break;
                                        }

                                        tem_zDis = zDis;
                                        NextZdis = 0;
                                        while( --DownLine >= 1)
                                        {
                                            tem_StartAngle = StartAngle - 20;
                                            tem_StartAngle = tem_StartAngle < 1? 1:tem_StartAngle;
                                            tem_EndAngle = EndAngle + 20;
                                            tem_EndAngle = tem_EndAngle > 1200 ? 1200:tem_EndAngle;

                                            if(line >= 1 && line <= 48)
                                            {
                                                if(DownLine >= 1 && DownLine <= 48)
                                                {

                                                }
                                                else if(DownLine >= 49 && DownLine <= 96)
                                                {
                                                    tem_StartAngle -= 38;
                                                    tem_EndAngle -= 38;
                                                }
                                                else if(DownLine >= 97 && DownLine <= 144)
                                                {

                                                }
                                            }
                                            else if(line >= 49 && line <= 96)
                                            {
                                                if(DownLine >= 1 && DownLine <= 48)
                                                {
                                                    tem_StartAngle += 38;
                                                    tem_EndAngle += 38;
                                                }
                                                else if(DownLine >= 49 && DownLine <= 96)
                                                {

                                                }
                                                else if(DownLine >= 97 && DownLine <= 144)
                                                {
                                                    tem_StartAngle += 38;
                                                    tem_EndAngle += 38;
                                                }
                                            }
                                            else if(line >= 97 && line <= 144)
                                            {
                                                if(DownLine >= 1 && DownLine <= 48)
                                                {

                                                }
                                                else if(DownLine >= 49 && DownLine <= 96)
                                                {
                                                    tem_StartAngle -= 38;
                                                    tem_EndAngle -= 38;
                                                }
                                                else if(DownLine >= 97 && DownLine <= 144)
                                                {

                                                }
                                            }
                                            if(tem_StartAngle < 1 || tem_StartAngle>=1200)
                                                break;
                                            if(tem_EndAngle < 1 || tem_EndAngle>=1200)
                                                break;

                                            int cn = 0;
                                            for(int j = tem_StartAngle; j < tem_EndAngle; j+=HResolution)
                                            {
                                                OnePointInfo& tem_point = points.GetData(1, DownLine, j);
                                                if(tem_point.distance == 0)
                                                    continue;
        //                                        if((std::abs(tem_zDis - tem_point.z) > 2) &&
        //                                            (j > tem_StartAngle+30) &&
        //                                            (j < tem_EndAngle - 30))
        //                                        {
        //                                            break;
        //                                        }

                                                if( (std::abs(dis - (int)tem_point.distance) <= 150) &&
                                                    (tem_point.LowPulseWidth < 16000) &&
                                                    (tem_point.NonFilterable == 0))
                                                {
                                                    tem_point.Expansion = 1;

                                                    NextZdis += tem_point.z;
                                                    cn++;

                                                    // if(ExpansionHighlinght)
                                                    // {
                                                    //     tem_point.r = 255;
                                                    //     tem_point.g = 0;
                                                    //     tem_point.b = 255;
                                                    // }
                                                    // else
                                                    {
                                                        tem_point.x = 0;
                                                        tem_point.y = 0;
                                                        tem_point.z = 0;
                                                        tem_point.distance = 0;
                                                    }
                                                }
                                            }

                                            if(cn > 0)
                                            {
                                                NextZdis /= cn;
                                                tem_zDis = NextZdis;
                                            }


                                            if(line-10 == DownLine)
                                                break;
                                        }
                                    }
                                }
                            }

                        }

                        sum = 0;
                        StartAngle = 0;
                        EndAngle = 0;
                        dis = 0;
                    }

                    PreAngle = tem_Points[i].azimuth;
                    PreHAngle = tem_Points[i].HAngle;
                }
            }

        }

        inline void Filterate::DoublePointAvg(OnePointInfo& src, OnePointInfo& p1,OnePointInfo& p2)
        {
            int LeftAddRightAvg = (p2.distance + p1.distance) /2;
            int LeftAddRightPWAvg = (p2.LowPulseWidth + p1.LowPulseWidth) /2;
            int intensity = (p2.intensity + p1.intensity) /2;
            double CosRideSin = COS((int)src.vazimuth) * SIN((int)src.azimuth * 10) *0.001;
            double CosRidCos = COS((int)src.vazimuth) * COS((int)src.azimuth * 10) *0.001;
            double SinRidThousandth = SIN((int)src.vazimuth) *0.001;
            src.distance = LeftAddRightAvg;
            src.LowPulseWidth = LeftAddRightPWAvg;
            src.intensity = intensity;
            src.y = src.distance * CosRideSin;//新坐标系计算方式，逆时针改顺时针的方式
            src.x = src.distance * CosRidCos;
            src.z =  src.distance * SinRidThousandth;//不补偿
        }
    
        inline void Filterate::Angle38Check(int& angle1, int& angle2, int cuLine, int checkLine)
        {
            if(cuLine >= 1 && cuLine <= 48)
            {
                if(checkLine >= 1 && checkLine <= 48)
                {

                }
                else if(checkLine >= 49 && checkLine <= 96)
                {
                    angle1 -= 38;
                    angle2 -= 38;
                }
                else if(checkLine >= 97 && checkLine <= 144)
                {

                }
            }
            else if(cuLine >= 49 && cuLine <= 96)
            {
                if(checkLine >= 1 && checkLine <= 48)
                {
                    angle1 += 38;
                    angle2 += 38;
                }
                else if(checkLine >= 49 && checkLine <= 96)
                {

                }
                else if(checkLine >= 97 && checkLine <= 144)
                {
                    angle1 += 38;
                    angle2 += 38;
                }
            }
            else if(cuLine >= 97 && cuLine <= 144)
            {
                if(checkLine >= 1 && checkLine <= 48)
                {

                }
                else if(checkLine >= 49 && checkLine <= 96)
                {
                    angle1 -= 38;
                    angle2 -= 38;
                }
                else if(checkLine >= 97 && checkLine <= 144)
                {

                }
            }
        }

        inline void Filterate::FilterNoiseV2(OnePointInfoManage &points,OnePointInfoManage &Prepoints)
        {
            std::vector<OnePointInfo> CheckPoint;
            // 只管5米后的点
            for(int line = 1; line <= 144;line++)
            {
                for(int angle = StartAngle; angle < 1200;angle+= HResolution * 2)
                {
                    OnePointInfo& Prepoint = Prepoints.GetData(1,line,angle);
                    OnePointInfo& point = points.GetData(1,line,angle);


                    if(point.distance < FullSearchScope)
                    {
                        continue;
                    }

                    if(point.line == 69 && point.HAngle == 755)
                    {
                        int debug = 0;
                    }

                    if(point.line == 0 || Prepoint.line == 0 || point.Time - Prepoint.Time > 300)
                    {
                        if(point.line != 0)
                        {
                            CheckPoint.push_back(point);
                        }
                        else
                        {
                            point.NonFilterable = 1;
                            if(HResolution == 1)
                            {
                                OnePointInfo& MatePoint = points.GetData(1,line,angle+1);
                                if(MatePoint.line != 0)
                                {
                                    MatePoint.NonFilterable = 1;
                                }
                            }
                        }

                        continue;
                    }

                    if( std::abs((long)point.distance - (long)Prepoint.distance) > 2500)
                    {
                        CheckPoint.push_back(point);
                    }
                    else
                    {
                        point.NonFilterable = 1;
                        if(HResolution == 1)
                        {
                            OnePointInfo& MatePoint = points.GetData(1,line,angle+1);
                            if(MatePoint.line != 0)
                            {
                                MatePoint.NonFilterable = 1;
                            }
                        }
                    }
                }
            }


            int cnt = 0;
            for(int i =0; i < (int)CheckPoint.size(); i++)
            {
                OnePointInfo& point = CheckPoint[i];
                int line = point.line;
                if(point.distance == 0)
                    continue;
                if(point.line == 69 && point.HAngle == 755)
                {
                    int debug = 0;
                }
                cnt = 0;
                // 查看上下5线
                int checkLineStart = (std::max)(point.line - 5, 1);
                int checkLineEnd = (std::min)(point.line + 5, 144);
                for(int checkLine = checkLineStart; checkLine < checkLineEnd; checkLine++)
                {

                    int FindLeftAngle = point.HAngle - 20;
                    int FindRightAngle = point.HAngle + 20;


                    if(line >= 1 && line <= 48)
                    {
                        if(checkLine >= 1 && checkLine <= 48)
                        {

                        }
                        else if(checkLine >= 49 && checkLine <= 96)
                        {
                            FindLeftAngle -= 38;
                            FindRightAngle -= 38;
                        }
                        else if(checkLine >= 97 && checkLine <= 144)
                        {

                        }
                    }
                    else if(line >= 49 && line <= 96)
                    {
                        if(checkLine >= 1 && checkLine <= 48)
                        {
                            FindLeftAngle += 38;
                            FindRightAngle += 38;
                        }
                        else if(checkLine >= 49 && checkLine <= 96)
                        {

                        }
                        else if(checkLine >= 97 && checkLine <= 144)
                        {
                            FindLeftAngle += 38;
                            FindRightAngle += 38;
                        }
                    }
                    else if(line >= 97 && line <= 144)
                    {
                        if(checkLine >= 1 && checkLine <= 48)
                        {

                        }
                        else if(checkLine >= 49 && checkLine <= 96)
                        {
                            FindLeftAngle -= 38;
                            FindRightAngle -= 38;
                        }
                        else if(checkLine >= 97 && checkLine <= 144)
                        {

                        }
                    }


                    FindLeftAngle = FindLeftAngle < StartAngle ? StartAngle : FindLeftAngle;
                    FindRightAngle = FindRightAngle > 1200? 1200 : FindRightAngle;

                    double OutlierRadiusPow = OutlierRadius * OutlierRadius;

                    for(int ag = FindLeftAngle; ag < FindRightAngle; ag += HResolution * 2)
                    {
                        if(ag == point.HAngle && point.line == checkLine)
                            continue;

                        OnePointInfo& tem = points.GetData(1,checkLine,ag);
                        if(tem.distance == 0)
                            continue;

        //                float pArray[] __attribute__ ((aligned (16))) = { 0, 0, 0, 0};
        //                pArray[0] = tem.x - point.x;
        //                pArray[1] = tem.y - point.y;
        //                pArray[2] = tem.z - point.z;
        //                DWORD dwGroupCount = 4 / 4;
        //                __m128 e_Scale = _mm_set_ps1(fScale);//设置所有4个值为同一值

        //                for (DWORD i = 0; i < dwGroupCount; i++)
        //                {
        //                    *(__m128*)(pArray + i * 4) = _mm_sqrt_ps(e_Scale);
        //                }


                        double val = (std::pow(tem.x - point.x,2) + std::pow(tem.y - point.y,2) + std::pow(tem.z - point.z,2));
                        if(val <= OutlierRadiusPow)
                        {
                            cnt++;
                            if(cnt >= OutlierNum)
                            {
                                points.GetData(1,line,point.HAngle).NonFilterable = 1;
                                if(HResolution == 1)
                                {
                                    OnePointInfo& MatePoint = points.GetData(1,line,point.HAngle+1);
                                    if(MatePoint.line != 0)
                                    {
                                        MatePoint.NonFilterable = 1;
                                    }
                                }
                                break;
                            }
                        }
                    }
                    if(cnt >= OutlierNum)
                        break;
                }
            }
        }
    
        inline void Filterate::FilterNoiseProcess(int init, OnePointInfo &point,OnePointInfo &Prepoint,OnePointInfoManage &points)
        {
            static std::vector<OnePointInfo> CheckPoint;
            if(!OutlierEnb)
                return;


            if(init == 1)
            {
                int cnt = 0;
                for(int i =0; i < (int)CheckPoint.size(); i++)
                {
                    OnePointInfo& point = CheckPoint[i];
                    int line = point.line;
                    if(point.distance == 0)
                        continue;

                    if(point.line == 126 && point.HAngle == 189)
                    {
                        int debug = 0;
                    }

                    cnt = 0;
                    // 查看上下5线
                    int checkLineStart = (std::max)(point.line - 5, 1);
                    int checkLineEnd = (std::min)(point.line + 5, 144);
                    for(int checkLine = checkLineStart; checkLine < checkLineEnd; checkLine++)
                    {

                        int FindLeftAngle = point.HAngle - 20;
                        int FindRightAngle = point.HAngle + 20;

                        Angle38Check(FindLeftAngle, FindRightAngle, line, checkLine);


                        FindLeftAngle = FindLeftAngle < StartAngle ? StartAngle : FindLeftAngle;
                        FindRightAngle = FindRightAngle > 1200? 1200 : FindRightAngle;

                        double OutlierRadiusPow = OutlierRadius * OutlierRadius;

                        for(int ag = FindLeftAngle; ag < FindRightAngle; ag += HResolution * 2)
                        {
                            if(ag == point.HAngle && point.line == checkLine)
                                continue;

                            OnePointInfo& tem = points.GetData(point.EchoType,checkLine,ag);
                            if(tem.distance == 0)
                                continue;

                            double val = (std::pow(tem.x - point.x,2) + std::pow(tem.y - point.y,2) + std::pow(tem.z - point.z,2));
                            if(val <= OutlierRadiusPow)
                            {
                                cnt++;
                                if(cnt >= OutlierNum)
                                {
                                    points.GetData(point.EchoType,line,point.HAngle).NonFilterable = 1;
                                    if(HResolution == 1)
                                    {
                                        OnePointInfo& MatePoint = points.GetData(point.EchoType,line,point.HAngle+1);
                                        if(MatePoint.line != 0)
                                        {
                                            MatePoint.NonFilterable = 1;
                                        }
                                    }
                                    break;
                                }
                            }
                        }
                        if(cnt >= OutlierNum)
                            break;
                    }
                }

                CheckPoint.clear();
                return;
            }


            if(StartAngle % 2 == 0)
            {
                if((int)point.HAngle % 2 != 0)
                {
                    return;
                }
            }
            else
            {
                if((int)point.HAngle % 2 == 0)
                {
                    return;
                }
            }

            if(point.distance < FullSearchScope)
            {
                return;
            }


            if(point.line == 0 || Prepoint.line == 0 || point.Time - Prepoint.Time > 300)
            {
                if(point.line != 0)
                {
                    CheckPoint.push_back(point);
                }
                else
                {
                    point.NonFilterable = 1;
                    if(HResolution == 1)
                    {
                        OnePointInfo& MatePoint = points.GetData(point.EchoType,point.line,point.HAngle+1);
                        if(MatePoint.line != 0)
                        {
                            MatePoint.NonFilterable = 1;
                        }
                    }
                }

                return;
            }

            if( std::abs((long)point.distance - (long)Prepoint.distance) > 2500)
            {
                CheckPoint.push_back(point);
            }
            else
            {
                point.NonFilterable = 1;
                if(HResolution == 1)
                {
                    OnePointInfo& MatePoint = points.GetData(point.EchoType,point.line,point.HAngle+1);
                    if(MatePoint.line != 0)
                    {
                        MatePoint.NonFilterable = 1;
                    }
                }
            }
        }

        // 近距离噪点
        inline void Filterate::CloseRangeOutliers(OnePointInfoManage &points, OnePointInfoManage &Prepoints)
        {

            int cnt = 0;
            for(int line = 1; line <= 144;line++)
            {
                for(int angle = StartAngle; angle < 1200;angle+=HResolution)
                {

                    OnePointInfo& point = points.GetData(1,line,angle);

                    if(point.distance < FullSearchScope)
                    {

                        if(point.distance == 0)
                            continue;


                        cnt = 0;

                        // 查看上下5线
                        int checkLineStart = (std::max)(point.line - 5, 1);
                        int checkLineEnd = (std::min)(point.line + 5, 144);
                        for(int checkLine = checkLineStart; checkLine < checkLineEnd; checkLine++)
                        {

                            int FindLeftAngle = point.HAngle - 20;
                            int FindRightAngle = point.HAngle + 20;


                            if(line >= 1 && line <= 48)
                            {
                                if(checkLine >= 1 && checkLine <= 48)
                                {

                                }
                                else if(checkLine >= 49 && checkLine <= 96)
                                {
                                    FindLeftAngle -= 38;
                                    FindRightAngle -= 38;
                                }
                                else if(checkLine >= 97 && checkLine <= 144)
                                {

                                }
                            }
                            else if(line >= 49 && line <= 96)
                            {
                                if(checkLine >= 1 && checkLine <= 48)
                                {
                                    FindLeftAngle += 38;
                                    FindRightAngle += 38;
                                }
                                else if(checkLine >= 49 && checkLine <= 96)
                                {

                                }
                                else if(checkLine >= 97 && checkLine <= 144)
                                {
                                    FindLeftAngle += 38;
                                    FindRightAngle += 38;
                                }
                            }
                            else if(line >= 97 && line <= 144)
                            {
                                if(checkLine >= 1 && checkLine <= 48)
                                {

                                }
                                else if(checkLine >= 49 && checkLine <= 96)
                                {
                                    FindLeftAngle -= 38;
                                    FindRightAngle -= 38;
                                }
                                else if(checkLine >= 97 && checkLine <= 144)
                                {

                                }
                            }


                            FindLeftAngle = FindLeftAngle < StartAngle ? StartAngle : FindLeftAngle;
                            FindRightAngle = FindRightAngle > 1200? 1200 : FindRightAngle;

                            double square = CloseRangeOutliersRadius * CloseRangeOutliersRadius;
                            for(int ag = FindLeftAngle; ag < FindRightAngle; ag+=HResolution * 2)
                            {
                                if(ag == point.HAngle && point.line == checkLine)
                                    continue;

                                OnePointInfo& tem = points.GetData(1,checkLine,ag);
                                if(tem.distance == 0)
                                    continue;
                                double xDiff = tem.x - point.x;
                                double yDiff = tem.y - point.y;
                                double zDiff = tem.z - point.z;
                                double val = (std::pow(xDiff,2) + std::pow(yDiff,2) + std::pow(zDiff,2));
                                if(val <= square)
                                {
                                    if(!(std::abs(xDiff) < 0.01 && std::abs(yDiff) < 0.01 && std::abs(zDiff) < 0.01))
                                    {
                                        cnt++;
                                        if(cnt >= CloseRangeOutliersNum)
                                        {
                                            point.CloseRangeOutliers = 1;

                                            break;
                                        }
                                    }

                                }
                            }
                            if(cnt >= CloseRangeOutliersNum)
                                break;
                        }

                    }
                }
            }

        }
    
        inline void Filterate::CloseRangeOutliersProcess(int init, OnePointInfo& point,OnePointInfoManage &points)
        {
            static int cnt = 0;

            if(!CloseRangeOutliersEnb)
                return;

            if(point.distance < FullSearchScope)
            {
                if(point.distance == 0)
                    return;

                cnt = 0;

                // 查看上下5线
                int checkLineStart = (std::max)(point.line - 5, 1);
                int checkLineEnd = (std::min)(point.line + 5, 144);
                for(int checkLine = checkLineStart; checkLine < checkLineEnd; checkLine++)
                {

                    int FindLeftAngle = point.HAngle - 20;
                    int FindRightAngle = point.HAngle + 20;

                    Angle38Check(FindLeftAngle,FindRightAngle,point.line,checkLine);

                    FindLeftAngle = FindLeftAngle < StartAngle ? StartAngle : FindLeftAngle;
                    FindRightAngle = FindRightAngle > 1200? 1200 : FindRightAngle;

                    double square = CloseRangeOutliersRadius * CloseRangeOutliersRadius;
                    for(int ag = FindLeftAngle; ag < FindRightAngle; ag+=HResolution * 2)
                    {
                        if(ag == point.HAngle && point.line == checkLine)
                            continue;

                        OnePointInfo& tem = points.GetData(point.EchoType,checkLine,ag);
                        if(tem.distance == 0)
                            continue;
                        double xDiff = tem.x - point.x;
                        double yDiff = tem.y - point.y;
                        double zDiff = tem.z - point.z;
                        double val = (std::pow(xDiff,2) + std::pow(yDiff,2) + std::pow(zDiff,2));
                        if(val <= square)
                        {
                            if(!(std::abs(xDiff) < 0.01 && std::abs(yDiff) < 0.01 && std::abs(zDiff) < 0.01))
                            {
                                cnt++;
                                if(cnt >= CloseRangeOutliersNum)
                                {
                                    point.CloseRangeOutliers = 1;

                                    break;
                                }
                            }
                        }
                    }
                    if(cnt >= CloseRangeOutliersNum)
                        break;
                }
            }

            if(init == 1)
            {
                cnt = 0;
            }
        }
    }
}

