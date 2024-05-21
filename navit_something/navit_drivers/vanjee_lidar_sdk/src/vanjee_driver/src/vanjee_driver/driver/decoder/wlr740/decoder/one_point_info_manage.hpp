#pragma once 
#include <vector>
#include <cassert>

//单点信息结构体
typedef struct OnePointInfo
{
    unsigned short line = 0;       //线号 1-144
    unsigned short Mirror = 0;     //转镜号
    unsigned short Channal = 0;   //LD号 1-48
    double azimuth = 0;    //方位角 (0~360)*100
    double vazimuth = 0;
    double HAngle = 0;     // 协议里的原始角度
    double VAngle = 0;     // 垂直角度
    unsigned int old_distance = 0;
    unsigned int distance = 0;   //距离(毫米)
    unsigned char intensity = 0;   //反射率
    unsigned short LowPulseWidth = 0;  // 低阈值脉宽
    unsigned short TallPulseWidth = 0; // 高阈值脉宽
    unsigned int index = 0;      //此点在1圈数据中的位置，通过方位角获取当前包所在标号.eg:732/10HZ:包数（0~179）*10*32+块数(0~9)*32+线数(0~31);
    int EchoType = 0;       //回波类型 1- 最强 2最后
    double Time = 0;
    int GroundFlag = 0;     // 地面点标志
    int GhostFlag = 0;      // 鬼影标志
    int StrongWeak = 0;     // 强弱发光标志 0：弱 1：强
    int label = 0;   // 聚类标签
    int scanindex = -1;          // 网格索引点
    int scanline = -1;
    unsigned int src_index = -1; //  原始索引
    double x = 0;										//x轴坐标，单位米
    double y = 0;										//y轴坐标，单位米
    double z = 0;										//z轴坐标，单位米
    int BlankPointCompen = 0;
    int _40BlankPointCompen = 0;
    int NonFilterable = 0;      // 0-不显示该点 1-显示该点
    int Expansion = 0;
    int CloseRangeOutliers = 0; // 0-不显示该点 1-显示该点
    float r = 0;//R/255
    float g = 0;//G/255
    float b = 0;//B/255
}OnePointInfo;


class ItIntexRecord
{
public:
    long long IndexRecord = -1;
    long long ItStrongIndexRecord = -1;
    long long ItLastIndexRecord = -1;
};

class OnePointInfoManage
{
private:
    std::vector<OnePointInfo> StrongPointData;
    std::vector<std::vector<long>> StrongPointDataIndex;


    std::vector<OnePointInfo> LastPointData;
    std::vector<std::vector<long>> LastPointDataIndex;

    int LineCount = 0;

    std::vector<ItIntexRecord> ItIndexRecords;

public:
    size_t StrongPointDataSize = 0;
    size_t LastPointDataSize = 0;

    // 由于采用size_t size() const;函数获取size会导致创建堆栈的耗时 因此将indexRecordsize设置为全局变量
    size_t indexRecordsize = 0; 
    OnePointInfoManage();
    ~OnePointInfoManage();
    OnePointInfoManage(int LineNo);
    // 复制构造函数
    OnePointInfoManage(const OnePointInfoManage& s);
    OnePointInfoManage(OnePointInfoManage&& s);
    // 等于号重载
    OnePointInfoManage& operator= (const OnePointInfoManage& s);
    OnePointInfoManage& operator= (OnePointInfoManage&& s);

    void Init(int LineNo);
    void insert(unsigned int index, OnePointInfo& Data);
    void insert(unsigned int index, OnePointInfo&& Data);
    void CleanLineData(unsigned int lineNo);
    // EchoType 0最强一重点云 1最后一重点云    LineNo 线号 0- 143 HAngle   水平角度 0-3600
    OnePointInfo& GetData(int EchoType,unsigned int LineNo,unsigned int HAngle);
    OnePointInfo& GetData(unsigned int index);
    long GetIndex(int EchoType,unsigned int LineNo,unsigned int HAngle);
    //OnePointInfo& GetDataConst(unsigned int index) const;
    OnePointInfo& GetDataIt(unsigned int index);           // 按迭代获取数据
    //uint GetIterationIndex(unsigned int index) const;      // 获取迭代索引号
    //OnePointInfo& GetDataItConst(unsigned int index) const;

    void Clear();
    size_t size() const;
    size_t sizeMax() const;
    std::vector<OnePointInfo>& Getorigndata();
};




//-----------------------------------------------------
OnePointInfoManage::OnePointInfoManage(int lineNo)
{
    indexRecordsize = 0;
    StrongPointDataSize = 0;
    LastPointDataSize = 0;
    LineCount = lineNo;
}

OnePointInfoManage::OnePointInfoManage()
{
}
OnePointInfoManage::~OnePointInfoManage()
{
}

OnePointInfoManage::OnePointInfoManage(OnePointInfoManage&& s)
{
    StrongPointData = std::move(s.StrongPointData);
    LastPointData = std::move(s.LastPointData);
    StrongPointDataIndex = std::move(s.StrongPointDataIndex);
    LastPointDataIndex = std::move(s.LastPointDataIndex);
    ItIndexRecords = std::move(s.ItIndexRecords);
    StrongPointDataSize = s.StrongPointDataSize;
    LastPointDataSize = s.LastPointDataSize;
    LineCount = s.LineCount;
    indexRecordsize = s.indexRecordsize;
}

OnePointInfoManage::OnePointInfoManage(const OnePointInfoManage& s)
{
    Init(s.LineCount);
    StrongPointData = s.StrongPointData;
    LastPointData = s.LastPointData;
    StrongPointDataIndex = s.StrongPointDataIndex;
    LastPointDataIndex = s.LastPointDataIndex;
    StrongPointDataSize = s.StrongPointDataSize;
    LastPointDataSize = s.LastPointDataSize;
    LineCount = s.LineCount;
    indexRecordsize = s.indexRecordsize;
    ItIndexRecords = s.ItIndexRecords;
}

// 等于号重载
OnePointInfoManage& OnePointInfoManage::operator= (const OnePointInfoManage& s){
    Init(s.LineCount);
    StrongPointData = s.StrongPointData;
    LastPointData = s.LastPointData;
    StrongPointDataIndex = s.StrongPointDataIndex;
    LastPointDataIndex = s.LastPointDataIndex;
    StrongPointDataSize = s.StrongPointDataSize;
    LastPointDataSize = s.LastPointDataSize;
    LineCount = s.LineCount;
    indexRecordsize = s.indexRecordsize;
    ItIndexRecords = s.ItIndexRecords;
    return *this;
}

OnePointInfoManage& OnePointInfoManage::operator= (OnePointInfoManage&& s){
    StrongPointData = std::move(s.StrongPointData);
    LastPointData = std::move(s.LastPointData);
    StrongPointDataIndex = std::move(s.StrongPointDataIndex);
    LastPointDataIndex = std::move(s.LastPointDataIndex);
    ItIndexRecords = std::move(s.ItIndexRecords);
    StrongPointDataSize = s.StrongPointDataSize;
    LastPointDataSize = s.LastPointDataSize;
    LineCount = s.LineCount;
    indexRecordsize = s.indexRecordsize;
    return *this;
}

void  OnePointInfoManage::Init(int lineNo)
{
    //auto start = std::chrono::steady_clock::now();
    for(size_t i = 0;i< 144;i++)
    {
        std::vector<long> v, v2;
        StrongPointDataIndex.push_back(v);
        LastPointDataIndex.push_back(v2);
        for(size_t j = 0; j < 3600;j++)
        {
            StrongPointDataIndex[i].push_back(-1);
            LastPointDataIndex[i].push_back(-1);
        }
    }
    StrongPointDataSize = 0;
    LastPointDataSize = 0;
    LineCount = lineNo;
    indexRecordsize = 0;
}

void OnePointInfoManage::Clear()
{
    StrongPointData.clear();
    LastPointData.clear();
    StrongPointDataIndex.clear();
    LastPointDataIndex.clear();
    ItIndexRecords.clear();
    for(size_t i = 0;i< 144;i++)
    {
        std::vector<long> v(3600,-1), v2(3600,-1);

        StrongPointDataIndex.push_back(v);
        LastPointDataIndex.push_back(v2);

//        for(size_t j = 0; j < 3600;j++)
//        {
//            StrongPointDataIndex[i].push_back(-1);
//            LastPointDataIndex[i].push_back(-1);
//        }
    }
    StrongPointDataSize = 0;
    LastPointDataSize = 0;
    indexRecordsize = 0;
}


inline void OnePointInfoManage::insert(unsigned int index, OnePointInfo& Data)
{
    //Q_ASSERT(PointData);
    if(Data.EchoType == 1)
    {
        if(StrongPointDataIndex[Data.line - 1][Data.HAngle] == -1)
        {
            StrongPointDataIndex[Data.line - 1][Data.HAngle] = StrongPointDataSize;
            StrongPointData.push_back(Data);
            ItIntexRecord rtem;
            rtem.IndexRecord = indexRecordsize;
            rtem.ItStrongIndexRecord = StrongPointData.size() - 1;
            ItIndexRecords.push_back(rtem);
            StrongPointDataSize++;
            indexRecordsize++;
        }
        else// 不包含重复插入的数据
        {
            StrongPointData[StrongPointDataIndex[Data.line - 1][Data.HAngle]] = Data;
        }
    }
    else
    {
        if(LastPointDataIndex[Data.line - 1][Data.HAngle] == -1)
        {
            LastPointDataIndex[Data.line - 1][Data.HAngle] = LastPointDataSize;
            LastPointData.push_back(Data);
            ItIntexRecord rtem;
            rtem.IndexRecord = indexRecordsize;
            rtem.ItLastIndexRecord = LastPointData.size() - 1;
            ItIndexRecords.push_back(rtem);
            LastPointDataSize++;
            indexRecordsize++;
        }
        else// 不包含重复插入的数据
        {
            LastPointData[LastPointDataIndex[Data.line - 1][Data.HAngle]] = Data;
        }
    }

}
inline void OnePointInfoManage::insert(unsigned int index, OnePointInfo&& Data)
{
    if(Data.EchoType == 1)
    {
        if(StrongPointDataIndex[Data.line - 1][Data.HAngle] == -1)
        {
            StrongPointDataIndex[Data.line - 1][Data.HAngle] = StrongPointDataSize;
            StrongPointData.push_back(Data);
            ItIntexRecord rtem;
            rtem.IndexRecord = indexRecordsize;
            rtem.ItStrongIndexRecord = StrongPointData.size() - 1;
            ItIndexRecords.push_back(rtem);
            StrongPointDataSize++;
            indexRecordsize++;
        }
        else// 不包含重复插入的数据
        {
            StrongPointData[StrongPointDataIndex[Data.line - 1][Data.HAngle]] = Data;
        }
    }
    else
    {
        if(LastPointDataIndex[Data.line - 1][Data.HAngle] == -1)
        {
            LastPointDataIndex[Data.line - 1][Data.HAngle] = LastPointDataSize;
            LastPointData.push_back(Data);
            ItIntexRecord rtem;
            rtem.IndexRecord = indexRecordsize;
            rtem.ItLastIndexRecord = LastPointData.size() - 1;
            ItIndexRecords.push_back(rtem);
            LastPointDataSize++;
            indexRecordsize++;
        }
        else// 不包含重复插入的数据
        {
            LastPointData[LastPointDataIndex[Data.line - 1][Data.HAngle]] = Data;
        }
    }
}

void OnePointInfoManage::CleanLineData(unsigned int lineNo)
{
    assert(lineNo-1 < LineCount);

    std::vector<long> v(3600,-1), v2(3600,-1);

//    for(int i = 0; i < 3600;i++)
//    {
//        if(StrongPointDataIndex[lineNo - 1][i] != -1)
//        {
//            for(int j = 0; j <ItIndexRecords.size();j++)
//            {
//                if(ItIndexRecords[j].ItStrongIndexRecord == StrongPointDataIndex[lineNo - 1][i])
//                {
//                    ItIndexRecords.removeAt(j);
//                    indexRecordsize--;
//                    break;
//                }
//            }
//            StrongPointDataIndex[lineNo - 1][i] = -1;
//        }
//        if(LastPointDataIndex[lineNo - 1][i] != -1)
//        {
//            for(int j = 0; j <ItIndexRecords.size();j++)
//            {
//                if(ItIndexRecords[j].ItLastIndexRecord == LastPointDataIndex[lineNo - 1][i])
//                {
//                    ItIndexRecords.removeAt(j);
//                    indexRecordsize--;
//                    break;
//                }
//            }
//            LastPointDataIndex[lineNo - 1][i] = -1;
//        }
//    }

    StrongPointDataIndex[lineNo - 1] = (v);
    LastPointDataIndex[lineNo - 1] = (v2);
}

OnePointInfo res;
inline OnePointInfo& OnePointInfoManage::GetData(int EchoType,unsigned int LineNo,unsigned int HAngle)
{
    assert(EchoType < 3);
    assert(LineNo-1 < LineCount);
    if(HAngle > 3600)
    {
        int vv = 0;
    }
    assert(HAngle <= 3600);
    if(EchoType == 1)
    {
        if(StrongPointDataIndex[LineNo - 1][HAngle] >= 0)
        {
            return StrongPointData[StrongPointDataIndex[LineNo-1][HAngle]];
        }
    }
    else
    {
        if(LastPointDataIndex[LineNo - 1][HAngle] >= 0)
        {
            return LastPointData[LastPointDataIndex[LineNo-1][HAngle]];
        }
    }
    return res;
}

inline OnePointInfo& OnePointInfoManage::GetData(unsigned int index)
{
    int EchoType = (index >= 144 *3600) ? 2:1;
    int Line = (EchoType == 1) ? (index / 3600):((index - (144 *3600)) / 3600);
    int HAngle = (EchoType == 1) ? (index % 3600):((index - (144 *3600)) % 3600);

    if(EchoType == 1)
    {
        if(StrongPointDataIndex[Line][HAngle] >= 0)
        {
            return StrongPointData[StrongPointDataIndex[Line][HAngle]];
        }
    }
    else
    {
        if(LastPointDataIndex[Line][HAngle] >= 0)
        {
            return LastPointData[LastPointDataIndex[Line][HAngle]];
        }
    }
    return res;
}
long OnePointInfoManage::GetIndex(int EchoType,unsigned int LineNo,unsigned int HAngle)
{
    assert(EchoType < 3);
    assert(LineNo <= LineCount);
    assert(HAngle <= 3600);
    if(EchoType == 1)
    {
        return StrongPointDataIndex[LineNo - 1][HAngle];
    }
    else
    {
        return LastPointDataIndex[LineNo - 1][HAngle];
    }
    return -1;
}

//uint OnePointInfoManage::GetIndex(unsigned int index)
//{
//    size_t size = PointDataIndex.size();
//    for(size_t i = 0; i < size; i++)
//    {
//        if(PointDataIndex[i] == index)
//        {
//            return i;
//        }
//    }
//    Q_ASSERT(false);
//    return 0;
//}

OnePointInfo& OnePointInfoManage::GetDataIt(unsigned int index)
{
    assert(index < indexRecordsize);

    ItIntexRecord rtem = ItIndexRecords[index];

    if(rtem.ItStrongIndexRecord != -1)
    {
        return StrongPointData[rtem.ItStrongIndexRecord];
    }
    else if(rtem.ItLastIndexRecord != -1)
    {
        return LastPointData[rtem.ItLastIndexRecord];
    }

    return res;
}


size_t OnePointInfoManage::size() const
{
    return indexRecordsize;
}

std::vector<OnePointInfo>& OnePointInfoManage::Getorigndata()
{
    return StrongPointData;
}