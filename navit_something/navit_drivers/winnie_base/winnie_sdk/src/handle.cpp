
#include <winnie_sdk/dispatch/execution.h>
#include <winnie_sdk/dispatch/handle.h>
#include <winnie_sdk/hardware/serial_device.h>
#include <winnie_sdk/hardware/udpm_device.h>
#include <winnie_sdk/protocol/protocol.h>

std::vector<std::string> split(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    while ((end = str.find(delimiter, start)) != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + delimiter.length();
    }
    tokens.push_back(str.substr(start));
    return tokens;
}

namespace winnie_sdk {

Handle::Handle(std::string device_url) : device_url_(std::move(device_url)), init_(false) {
    std::vector<std::string> result = split(device_url_, ":");
    if (result.size() >= 2) {
        std::string& device_type = result[0];
        std::string device_addr  = result[1];
        if (device_type == "serial") {
            int band = result.size() >= 3 ? std::stoi(result[2]) : 115200;
            device_  = std::make_shared<SerialDevice>(device_addr, band);
        } else if (device_type == "udpm") {
            int port = result.size() >= 3 ? std::stoi(result[2]) : 10000;
            device_  = std::make_shared<UdpmDevice>(device_addr.c_str(), port);
        }

        if (!device_->Init()) {
            device_.reset();
        }

    } else {
        printf("error NodeHandle!\n");
    }
}

bool Handle::Init() {
    if (!device_) {
        std::cout << "Can not open device: " << device_url_
                  << ". Please check if the device is inserted and connection is configured correctly!" << std::endl;
        return false;
    }

    std::cout << "Connection to " << device_url_ << std::endl;
    protocol_ = std::make_shared<Protocol>(device_);
    if (!protocol_->Init()) {
        std::cout << "Protocol initialization failed." << std::endl;
        return false;
    }

    executor_ = std::make_shared<Executor>(shared_from_this());
    std::cout << "Initialization of protocol layer and dispatch layer succeeded. " << std::endl;
    init_ = true;
    return true;
}

std::shared_ptr<Protocol>& Handle::GetProtocol() { return protocol_; }

void Handle::Spin() {
    if (!init_) {
        std::cout << "Handle is not initialized or error occurred during initialization." << std::endl;
        return;
    }
    for (const auto& sub : subscription_factory_) {
        executor_->ExecuteSubscription(sub);
    }
    for (const auto& client : client_factory_) {
        executor_->ExecuteClient(client);
    }
    for (const auto& service : service_factory_) {
        executor_->ExecuteService(service);
    }
}
}  // namespace winnie_sdk
