#include "integrated_navigation/driver/ins_sensor_driver.h"
#include "integrated_navigation/driver/gnss/novatel_gps_driver.h"

namespace integrated_navigation {
    ins_sensor_driver::ins_sensor_driver(ros::NodeHandle &nh)
        : rate_(200),
          reconnect_delay_s_(0.5),
          imu_frame_(0),
          novatel_frame_(0) {

        // Load yaml file parameters.
        nh.getParam("/multi_sensor_config/connection_type", connection_type_);
        LOG(INFO) << "connection type = " << connection_type_<<std::endl; 
        remain_str=std::string("");
        cout_time=0;

        if(connection_type_ == "serial"){
            std::string port_;
            int baudrate_;
            nh.getParam("/multi_sensor_config/baudrate", baudrate_);
            nh.getParam("/multi_sensor_config/port", port_);

            //设置串口属性，并打开串口
            ser.setPort(port_);
            ser.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
            ser.setTimeout(to);
            ser.open();
            ser.flushInput(); // first clear the buffer

            //io_common_ = std::make_shared<serial_common>(nh);
            LOG(INFO) << "IO type: serial"<<std::endl;
        }else if(connection_type_ == "tcp"){
            io_common_ = std::make_shared<tcp_common>(nh);
            LOG(INFO) << "IO type: tcp"<<std::endl;
        }else{
            LOG(ERROR) << "Invalid connection type ." << std::endl;
            return;
        }
        gnss_ = std::make_shared<novatel_gps_driver>(nh, io_common_);

        thread_ = boost::thread(&ins_sensor_driver::Spin, this);
    }

    ins_sensor_driver::~ins_sensor_driver() {
        io_common_->Disconnect();
    }

    void ins_sensor_driver::Spin() {
        ros::WallRate rate(rate_);
        
        while (ros::ok()) {
            if(ser.isOpen()){
                 MultiSensorDataFlow();
            }
            else{ // Could not connect to the device
                LOG(ERROR) << "Error connecting to device: " << io_common_->get_device().c_str() << std::endl;
            }
            // Sleep for a microsecond to prevent CPU hogging
            rate.sleep();
        } // While (ros::ok) (outer loop to reconnect to device)

        io_common_->Disconnect();
    }
   //去掉所有空格
    void trim_all(std::string &s)
    {
        int index = 0;
        if( !s.empty())
        {
            while( (index = s.find(' ',index)) != std::string::npos)
            {
                s.erase(index,1);
            }
        }
    }
   std::vector<std::string>split_string(std::string str,std::string& remain_str) {
	std::vector<int>indexs;
	int pos = 0;
    std::string target("$");
	while ((pos = str.find(target, pos)) != std::string::npos) {
		indexs.push_back(pos);
		pos++; // 继续查找下一个位置
	}
    target=std::string("#");
    pos = 0;
    while ((pos = str.find(target, pos)) != std::string::npos) {
		indexs.push_back(pos);
		pos++; // 继续查找下一个位置
	}
    std::sort(indexs.begin(),indexs.end());
	std::vector<std::string> res;
    remain_str.clear();
    remain_str=std::string("");
    
    if( indexs.size()<2)
    {
        remain_str=str;
        return res;
    }
      
	for (int i = 0; i < indexs.size() - 1; i++) {
        std::string tmp_str=str.substr(indexs[i], indexs[i+1]-indexs[i]);
        if((tmp_str.find("$")!=std::string::npos||tmp_str.find("#")!=std::string::npos)&&tmp_str.find("\r\n")!=std::string::npos
        &&tmp_str.find("*")!=std::string::npos)
           { trim_all(tmp_str);
		    res.push_back(str.substr(indexs[i], indexs[i+1]-indexs[i]));}
	}
   
    std::string sub=str.substr(indexs[indexs.size()-1],str.length()-indexs[indexs.size()-1]);
    remain_str=str.substr(indexs[indexs.size()-1],str.length()-indexs[indexs.size()-1]);
	return res;
}
 void clear_str(std::string &sentence){
    std::string tmp="";
    for(int i=0;i<sentence.length();i++){
        if (sentence[i] == 9 || sentence[i] == 10 || sentence[i] == 11 || sentence[i] == 13 ||
          (sentence[i] >= 32 && sentence[i] <= 126))
          tmp+=sentence[i];
    }
    sentence=tmp;
 }

    void ins_sensor_driver::MultiSensorDataFlow() {
        memset(rbuf, 0, sizeof(rbuf));
        try {
            numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
        }
        catch (serial::IOException &e) {
            ROS_ERROR_STREAM("Port crashed！ Please check cable!");
        }
        if (numinbuf > 0) {
            numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
        if (numgetted == numinbuf){
            rbuf_str.clear();
            rbuf_str=std::string("");
            rbuf_str=remain_str;
           // ROS_INFO("remain_str:%s",remain_str);
            rbuf_str.insert(rbuf_str.end(),rbuf,rbuf+2048);
            clear_str(rbuf_str);
            r_strs.resize(0);
            r_strs=split_string(rbuf_str,remain_str);
            data_buffer_.push_back(uint8_t(0));
            for(std::string str:r_strs)
            {
                if(str.find(std::string("UNIHEADINGA"))!=std::string::npos||
                //||str.find(std::string("GNRMC"))!=std::string::npos
                str.find(std::string("BESTNAVA"))!=std::string::npos ||
                str.find(std::string("BESTNAVXYZA"))!=std::string::npos)
                  {
                    //std::cout<<str;
                     std::vector<uint8_t> tmp=data_buffer_;
                     gnss_->novatel_data_callback(tmp,str);
                     cout_time=0;
                  } 
            }
        } 
            
        }
        if(cout_time++>1000){
           ROS_ERROR_STREAM("RTK serial port receiving data timeout!!!");
        }
 }

    

    bool ins_sensor_driver::read_from_connector() {
        data_buffer_.clear();
        io_common::ReadResult read_result = io_common::ReadResult::READ_ERROR;
        
        read_result = io_common_->IORead(data_buffer_, 0);
        switch (read_result){
            case io_common::ReadResult::READ_INSUFFICIENT_DATA :{
                LOG(WARNING) << "Buffer data is not enough!";
                break;
            }
            case io_common::ReadResult::READ_TIMEOUT :{
                LOG(WARNING) << "Read timeout!";
                break;
            }
            case io_common::ReadResult::READ_INTERRUPTED :{
                LOG(WARNING) << "Read interrupted!";
                break;
            }
            case io_common::ReadResult::READ_ERROR :{
                LOG(WARNING) << "Read error!";
                break;
            }
            case io_common::ReadResult::READ_PARSE_FAILED :{
                LOG(WARNING) << "Read parse failed!";
                break;
            }
            
            default:{
                // DLOG(WARNING) << "Read default!";
                break;
            }
        } 

        if(read_result == io_common::ReadResult::READ_SUCCESS){
            return true;
        }

        LOG(ERROR) << "Invalid connection type, Read Error.";
        return false;
    }
}