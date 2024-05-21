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
