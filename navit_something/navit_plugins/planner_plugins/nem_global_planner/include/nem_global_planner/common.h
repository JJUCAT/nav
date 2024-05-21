#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

const std::string JOY_KEY_LEFT  = "JOY_KEY_LEFT";
const std::string JOY_KEY_RIGHT = "JOY_KEY_RIGHT";
const std::string JOY_KEY_UP    = "JOY_KEY_UP";
const std::string JOY_KEY_DOWN  = "JOY_KEY_DOWN";
const std::string JOY_KEY_A     = "JOY_KEY_A";
const std::string JOY_KEY_B     = "JOY_KEY_B";
const std::string JOY_KEY_X     = "JOY_KEY_X";
const std::string JOY_KEY_Y     = "JOY_KEY_Y";
const std::string JOY_KEY_LB    = "JOY_KEY_LB";
const std::string JOY_KEY_LT    = "JOY_KEY_LT";
const std::string JOY_KEY_RB    = "JOY_KEY_RB";
const std::string JOY_KEY_RT    = "JOY_KEY_RT";
const std::string JOY_KEY_BACK  = "JOY_KEY_BACK";
const std::string JOY_KEY_START = "JOY_KEY_START";

const float FLOAT_OFFSET = 0.0001;
const float PTZ_OFFSET   = 1.0;

template <class T>
class CSingleton {
   public:
    static T &GetInstance() {
        if (t_instance.get() == NULL) {
            std::lock_guard<std::recursive_mutex> lck(t_mtx);
            if (t_instance.get() == NULL) {
                std::lock_guard<std::recursive_mutex> lck(t_mtx);
                t_instance.reset(new T());
            }
        }

        return *(t_instance.get());
    }

   protected:
    CSingleton() {}

    virtual ~CSingleton() {}

    static std::auto_ptr<T> t_instance;
    static std::recursive_mutex t_mtx;

   private:
    CSingleton(const CSingleton &other);

    CSingleton &operator=(const CSingleton &other);

    friend class std::auto_ptr<T>;
};

template <class T>
std::auto_ptr<T> CSingleton<T>::t_instance;

template <class T>
std::recursive_mutex CSingleton<T>::t_mtx;

#ifndef DECLARE_SINGLETON_CLASS
#define DECLARE_SINGLETON_CLASS(type) \
    friend class auto_ptr<type>;      \
    friend class CSingleton<type>;
#endif

class CTools : public CSingleton<CTools> {
   public:
    // sample
    // need add "cfgFile" param in launch file first EX: <param name="cfgFile" value="$(find base)/cfg/base.yaml"/>
    // CTools::DumpCfgFile("base_node", "joyXSpeed", 0.1);
    template <class T>
    static void DumpCfgFile(std::string node, std::string key, T value) {
        std::string strKey = "/" + node + "/" + key;
        std::cout << strKey << std::endl;

        ros::param::set(strKey, value);
        std::string strCfgFile = "/" + node + "/cfgFile";
        std::cout << strCfgFile << std::endl;
        if (ros::param::has(strCfgFile)) {
            std::string cfgFile("");
            if (ros::param::get(strCfgFile, cfgFile)) {
                std::cout << cfgFile << std::endl;
                std::string strCmd = "rosparam dump " + cfgFile + " " + node;
                system(strCmd.c_str());
            }
        }
    }

    // sample
    // double test;
    // CTools::GetParam("base_node", "joyXSpeed", test);
    // std::cout << test;
    template <class T>
    static bool GetParam(std::string node, std::string key, T &value) {
        std::string strKey = "/" + node + "/" + key;
        if (ros::param::has(strKey)) {
            std::cout << strKey << std::endl;
            if (ros::param::get(strKey, value)) {
                std::cout << value << std::endl;
                return true;
            }
        }
        return false;
    }

    template <class T>
    void AddParam(std::string node, std::string key, T *param, T defaultValue) {
        std::string strKey = "/" + node + "/" + key;
        m_mapParam[strKey] = (void *)param;
        if (GetParam<T>(node, key, *param)) {
            std::cout << *param << std::endl;
        } else {
            *param = defaultValue;
            ros::param::set(strKey, defaultValue);
        }
    }

    template <class T>
    bool SetParam(std::string node, std::string key, T value) {
        std::string strKey                         = "/" + node + "/" + key;
        std::map<std::string, void *>::iterator it = m_mapParam.find(strKey);
        if (it != m_mapParam.end()) {
            *((T *)(it->second)) = value;
            DumpCfgFile<T>(node, key, value);
            return true;
        }
        return false;
    }

    template <class T>
    bool SetParamOnly(std::string node, std::string key, T value) {
        std::string strKey                         = "/" + node + "/" + key;
        std::map<std::string, void *>::iterator it = m_mapParam.find(strKey);
        if (it != m_mapParam.end()) {
            *((T *)(it->second)) = value;
            ros::param::set(strKey, value);
            return true;
        }
        return false;
    }

   private:
    std::map<std::string, void *> m_mapParam;
};

#endif
