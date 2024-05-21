#ifndef NAVIT_UTILS_CPU_UTIL_H_
#define NAVIT_UTILS_CPU_UTIL_H_
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

namespace navit_utils {
constexpr int kVmrssLine = 22;
constexpr int kVmsizeLine = 13;
constexpr int kProcessItem = 14;

struct TotalCpuOccupy {
  uint64_t user;      // 用户态CPU时间
  uint64_t nice;      // 用于nice操作的CPU时间
  uint64_t system;    // 系统态CPU时间
  uint64_t idle;      // 空闲CPU时间
};

struct ProcCpuOccupy {
  uint32_t pid;       // 进程id
  uint64_t utime;     //用户态CPU时间
  uint64_t stime;     //系统态CPU时间
  uint64_t cutime;    //所有子进程用户态CPU时间
  uint64_t cstime;    //所有子进程系统态CPU时间
};

// 通过传入字符串buffer和item的位置，返回item在buffer中的位置指针
const char* getItems(const char* buffer, uint32_t item) {
  const char *p =buffer;
  int len = strlen(buffer);
  int count = 0;

  for (int i = 0; i < len; ++i) {
    if (' ' == *p) {
      count ++;
      if(count == item -1) {
        p++;
        break;
      }
    }
    p++;
  }
  return p;
}

// 获取指定进程的CPU占用时间
uint64_t getCpuProcOccupy(const uint32_t pid) {
  char file[64] = {0};
  ProcCpuOccupy t;
  FILE* fd;
  char line_buff[1024] = {0};
  sprintf(file,"/proc/%d/stat",pid);

  fd = fopen(file,"r");
  if (nullptr == fd) {
    return 0;
  }
  
  fgets(line_buff, sizeof(line_buff), fd);

  sscanf(line_buff,"%u",&t.pid);
  const char* q = getItems(line_buff, kProcessItem);
  sscanf(q,"%ld %ld %ld %ld",&t.utime,&t.stime,&t.cutime,&t.cstime);
  fclose(fd);

  return (t.utime + t.stime + t.cutime + t.cstime);
}

// 获取指定进程的内存占用情况
int getProcMemory(uint32_t pid) {
  char file[64] = {0};
  sprintf(file,"/proc/%d/status",pid);

  FILE* fd = fopen(file,"r");
  if(nullptr == fd) {
    return 0;
  }

  char line_buff[512] = {0};
  int vmrss = 0;
  for (int i = 0; i < kVmrssLine - 1; i++) {
    fgets(line_buff, sizeof(line_buff), fd);
  }
  fgets(line_buff, sizeof(line_buff), fd);
  sscanf(line_buff, "VmRSS: %d kB", &vmrss);
  fclose(fd);

  return vmrss;
}

// 获取系统的总内存
int getTotalMemory() {
  FILE* fd = fopen("/proc/meminfo","r");
  if(nullptr == fd) {
    return 0;
  }

  char line_buff[512] = {0};
  int memTotal = 0;
  fgets(line_buff, sizeof(line_buff), fd);
  sscanf(line_buff, "MemTotal: %d kB", &memTotal);
  fclose(fd);

  return memTotal;
}
int32_t getProcessPid(const std::string& process_name) {
  FILE *fp;
  char buf[100];
  char cmd[200] = {'\0'};
  pid_t pid = -1;
  sprintf(cmd, "pidof %s", process_name.data());

  if((fp = popen(cmd, "r")) != NULL) {
  	if(fgets(buf, 255, fp) != NULL) {
  	  pid = atoi(buf);
    }
  }
  pclose(fp);
  return pid;
}

// 获取一组进程的PID。参数process_name为进程名称。
// 如果找到对应的进程，将其PID加入到pids向量中，并返回true。否则，返回false。
bool getProcessPids(const std::string& process_name, std::vector<int32_t>* pids) {
  FILE *fp;
  char buf[100] = {'\0'};
  char cmd[200] = {'\0'};
  int32_t pid = -1;
  sprintf(cmd, "pidof %s", process_name.data());

  if((fp = popen(cmd, "r")) != NULL) {
    int begin = 0;
    if(fgets(buf, 255, fp) != NULL) {
      std::string pids_string = buf;
      for (size_t i = 0; buf[i + 1] != '\0'; ++i) {
        if (buf[i] < '0' || buf[i] > '9') {
          pid = atoi(pids_string.substr(begin, i - begin).c_str());
          pids->push_back(pid);
          begin = i + 1;
        }
      }
      pid = atoi(pids_string.substr(begin, pids_string.size() - begin + 1).c_str());
      pids->push_back(pid);
    } else {
      return false;
    }
  } else {
    return false;
  }
  pclose(fp);
  return true;
}

uint64_t getCpuTotalOccupy() {
  FILE *fd;
  char buff[1024] = {0};
  TotalCpuOccupy t;

  fd = fopen("/proc/stat","r");
  if (nullptr == fd) {
  	return 0;
  }

  fgets(buff, sizeof(buff), fd);
  char name[64] = {0};
  sscanf(buff, "%s %ld %ld %ld %ld", name, &t.user, &t.nice, &t.system, &t.idle);
  // std::cout << t.user << ", " << t.nice << ", " << t.system << ", " << t.idle << '\n';
  fclose(fd);
	
  return (t.user + t.nice + t.system + t.idle);
}

int getCpuTemperature() {
  FILE * fd;
  char name[32];
  int cpu_temp = 0;
  char line_buff[256];
  char file_prefix[128] = "/sys/class/hwmon/hwmon";
  char file_index[8] = "";
  char file_name[128] = "";

  for (int index = 0; index < 10; ++index) {
    file_name[0] = '\0';
    strcat(file_name, file_prefix);
    sprintf(file_index, "%d", index);
    strcat(file_name, file_index);
    strcat(file_name, "/name");

    if (access(file_name, F_OK) == 0) {
      fd = fopen(file_name, "r");
      fgets(line_buff, sizeof(line_buff), fd);
      sscanf(line_buff, "%s", name);
      fclose(fd);

      if (strcmp(name, "coretemp") == 0) {
        file_name[0] = '\0';
        strcat(file_name, file_prefix);
        sprintf(file_index, "%d", index);
        strcat(file_name, file_index);
        strcat(file_name, "/temp1_input");

        if (access(file_name, F_OK) == 0) {
          fd = fopen(file_name, "r");
          fgets(line_buff, sizeof(line_buff), fd);
          sscanf(line_buff, "%d", &cpu_temp);
          fclose(fd);
          return cpu_temp * 0.001;
        }
      }
    }
  }
  return 0;
}
int getTotalMemery() {
  char* file = (char*)"/proc/meminfo";
    
  FILE *fd;       
  char line_buff[256] = {0};                                                                                               
  fd = fopen (file, "r"); 
   
  char name[32];
  int memtotal;
  fgets (line_buff, sizeof(line_buff), fd);
  sscanf (line_buff, "%s %d", name,&memtotal);
  fclose(fd);     
  return memtotal;
}

}  // namespace navit_utils
#endif  // NAVIT_UTILS_SYSTEM_UTILS_H_