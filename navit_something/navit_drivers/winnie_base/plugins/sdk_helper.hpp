//
// Created by fan on 23-7-3.
//

#ifndef WINNIE_BASE_SDK_HELPER_HPP
#define WINNIE_BASE_SDK_HELPER_HPP

#include <winnie_sdk/winnie_sdk.h>

namespace winnie_base {

/// 未考虑相同的结构体，不同的cmd_set.
template <typename T>
class SdkPublisherHelper {
   public:
    void Init(winnie_sdk::Handle* handle, uint8_t cmd_set, uint8_t cmd_id, uint8_t sender, uint8_t receiver) {
        publisher_ = handle->CreatePublisher<T>(cmd_set, cmd_id, sender, receiver);
    }

    void publish(T& msg) { publisher_->Publish(msg); }

   private:
    std::shared_ptr<winnie_sdk::Publisher<T>> publisher_;
};

template <typename T>
class SdkSubscriberHelper {
   public:
    using CallBackType = std::function<void(const std::shared_ptr<T>&)>;

    void Init(winnie_sdk::Handle* handle, uint8_t cmd_set, uint8_t cmd_id, uint8_t sender, uint8_t receiver) {
        subscriber_ =
            handle->CreateSubscriber<T>(cmd_set, cmd_id, sender, receiver,
                                        std::bind(&SdkSubscriberHelper::on_received, this, std::placeholders::_1));
    }

    void register_topic_callback(const CallBackType& callback) {
        // std::cout << "register_topic_callback !" << std::endl;
        callback_ = callback;
    }

   private:
    void on_received(const std::shared_ptr<T>& msg) {
        // std::cout << "on_received !" << std::endl;
        if (callback_) {
            callback_(msg);
        }
    }
    CallBackType callback_;
    std::shared_ptr<winnie_sdk::Subscription<T>> subscriber_;
};

template <typename TRequest, typename TResponse>
class SdkServiceClientHelper {
   public:
    using Client         = winnie_sdk::Client<TRequest, TResponse>;
    using SharedFuture   = typename Client::SharedFuture;
    using SharedRequest  = typename Client::SharedRequest;
    using SharedResponse = typename Client::SharedResponse;

    void Init(winnie_sdk::Handle* handle, uint8_t cmd_set, uint8_t cmd_id, uint8_t sender,
                           uint8_t receiver) {
        client_ = handle->CreateClient<TRequest, TResponse>(cmd_set, cmd_id, sender, receiver);
    }

    bool call(SharedRequest& request, SharedResponse& response, int timeout_ms = 1000) {
        std::unique_lock<std::mutex> lock(mutex_);
        client_->AsyncSendRequest(request, [this](SharedFuture result) { on_response(result); });
        if (cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms)) == std::cv_status::timeout) {
            return false;
        }
        response = result_;
        return true;
    }

   private:
    void on_response(SharedFuture result) {
        std::unique_lock<std::mutex> lock(mutex_);
        result_ = result.get();
        cv_.notify_all();
    }

    std::mutex mutex_;
    std::condition_variable cv_;

    SharedResponse result_;
    std::shared_ptr<Client> client_;
};
}  // namespace winnie_base
#endif  // WINNIE_BASE_SDK_HELPER_HPP
