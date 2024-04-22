#include <navit_controller/controller_server.h>
#include <nodelet/nodelet.h>

namespace navit_controller
{
    /**
     * @brief Nodelet version of the ControllerServer class.
     */
    class ControllerServerNodelet : public nodelet::Nodelet
    {
    public:
        ControllerServerNodelet() {}
        ~ControllerServerNodelet() {}

    private:
        virtual void onInit();

        boost::shared_ptr<ControllerServer> controller_server_;
    };
}
