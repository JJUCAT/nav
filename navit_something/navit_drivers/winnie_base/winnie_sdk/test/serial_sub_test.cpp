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
