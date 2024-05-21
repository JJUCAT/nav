# 功能

解决上下位机的串口通讯问题

# 使用方式

- 串口设备 serial:/dev/ttyUSB0
- 网络设备 udpm:239.255.76.67:7667

ubuntu下虚拟串口
```shell
socat -d -d PTY PTY

2023/05/25 16:16:53 socat[42801] N PTY is /dev/pts/12
2023/05/25 16:16:53 socat[42801] N PTY is /dev/pts/13
2023/05/25 16:16:53 socat[42801] N starting data transfer loop with FDs [5,5] and [7,7]
则得到了两个虚拟连接的串口 /dev/pts/12 和 /dev/pts/13
```


1. 从指定串口订阅数据

```c++
#include <unistd.h>
#include <winnie_sdk/winnie_sdk.h>

#include "protocol_winnie.h"

void testInfoCallback(const std::shared_ptr<winnie_sdk::cmd_winnie_info> winnie_info) {
    std::cout << "height is : " << winnie_info->height << std::endl;
}

int main() {
    auto handle = std::make_shared<winnie_sdk::Handle>("serial:/dev/pts/13:115200");
    handle->Init();
    handle->CreateSubscriber<winnie_sdk::cmd_winnie_info>(WINNIE_CMD_SET, CMD_PUSH_WINNIE_INFO, JETSON_ADDRESS,
                                                          JETSON_ADDRESS,
                                                          std::bind(&testInfoCallback, std::placeholders::_1));

    while (1) {
        handle->Spin();
        usleep(1000);  // 10 ms
    }
    std::cout << "1111" << std::endl;
    return 0;
}

```

2. 发布数据到指定串口

```c++
#include <unistd.h>
#include <winnie_sdk/winnie_sdk.h>

#include "protocol_winnie.h"

int main() {
    auto handle = std::make_shared<winnie_sdk::Handle>("serial:/dev/pts/12:115200");
    handle->Init();
    auto pub = handle->CreatePublisher<winnie_sdk::cmd_winnie_info>(WINNIE_CMD_SET, CMD_PUSH_WINNIE_INFO,
                                                                    JETSON_ADDRESS, JETSON_ADDRESS);

    winnie_sdk::cmd_winnie_info msg;
    msg.height = 1.234;

    while (1) {
        pub->Publish(msg);
        printf("pub %f\n", msg.height);
        sleep(1);
        msg.height += 0.001;
    }
}

```

3. 从局域网订阅数据

```c++
#include <unistd.h>
#include <winnie_sdk/winnie_sdk.h>

#include "protocol_winnie.h"

void testInfoCallback(const std::shared_ptr<winnie_sdk::cmd_winnie_info> winnie_info) {
    std::cout << "height is : " << winnie_info->height << std::endl;
}

int main() {
    auto handle = std::make_shared<winnie_sdk::Handle>();
    handle->Init();
    handle->CreateSubscriber<winnie_sdk::cmd_winnie_info>(WINNIE_CMD_SET, CMD_PUSH_WINNIE_INFO, JETSON_ADDRESS,
                                                          JETSON_ADDRESS,
                                                          std::bind(&testInfoCallback, std::placeholders::_1));

    while (1) {
        handle->Spin();
        usleep(10000);  // 10 ms
    }
    std::cout << "1111" << std::endl;
    return 0;
}

```

4. 发布数据到局域网

```c++
#include <unistd.h>
#include <winnie_sdk/winnie_sdk.h>

#include "protocol_winnie.h"

int main() {
    auto handle = std::make_shared<winnie_sdk::Handle>();
    handle->Init();
    auto pub = handle->CreatePublisher<winnie_sdk::cmd_winnie_info>(WINNIE_CMD_SET, CMD_PUSH_WINNIE_INFO,
                                                                    JETSON_ADDRESS, JETSON_ADDRESS);

    winnie_sdk::cmd_winnie_info msg;
    msg.height = 1;

    while (1) {
        pub->Publish(msg);
        printf("pub %f\n", msg.height);
        sleep(1);
        msg.height += 1;
    }
}
```

