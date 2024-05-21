#ifndef WINNIE_SDK_EXECUTION_H
#define WINNIE_SDK_EXECUTION_H

#include <memory>

namespace winnie_sdk {
class Handle;

class SubscriptionBase;

class PublisherBase;

class ClientBase;

class ServiceBase;

/**
 * @brief Executor class to provide execute interface for subscriber, service server and client in the dispatch layer
 */
class Executor {
   public:
    /**
     * @brief Constructor of Executor
     * @param handle Pointer of handle which consists of handler of executor, protocol layer and hardware layer
     */
    explicit Executor(std::shared_ptr<Handle> handle);

    ~Executor() = default;

    /**
     * @brief Get the handle
     * @return Pointer of handle
     */
    std::shared_ptr<Handle> GetHandle();

    /**
     * @brief Given the subscription, invoke its callback function
     * @param subscription Subscription base_node.py pointer of certain command
     */
    void ExecuteSubscription(const std::shared_ptr<SubscriptionBase> &subscription);

    /**
     * @brief Given the service, invoke its callback function and call the protocol layer to send response/ack
     * @param service Service base_node.py pointer of certain command
     */
    void ExecuteService(const std::shared_ptr<ServiceBase> &service);

    /**
     * @brief Given the client, call the protocol layer to send command and wait for the response/ack
     * @param client Client base_node.py pointer of certain command
     */
    void ExecuteClient(const std::shared_ptr<ClientBase> &client);

   private:
    //! pointer of handle
    std::shared_ptr<Handle> handle_;
};
}  // namespace winnie_sdk
#endif  // WINNIE_SDK_EXECUTION_H
