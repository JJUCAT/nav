#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <rviz/panel.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <QButtonGroup>
#include <ui_custom_panel.h> 

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <navit_msgs/TaskCommand.h>
#include <navit_msgs/CmdRecordGeometry.h>
#include <navit_msgs/BuildGridMap.h>
#include <navit_common/coordinate_trans.h>
#include <vector>
#include <google/protobuf/util/json_util.h>

#include "message_navit_map.pb.h"

#include "navit_rviz_panel/rviz_point_with_line_tool.h"

namespace navit_rviz_panel
{
    class OperationToolsPanel : public rviz::Panel
    {
    Q_OBJECT
    public:
        /*@brief Constructor
        */
        OperationToolsPanel(QWidget * parent = 0);
        virtual void save(rviz::Config config) const;
        virtual void load(const rviz::Config & config);

        inline static constexpr const char TOPIC_MAP_INFO[] = "/navit/map_info";
        inline static constexpr const char TOPIC_MAP_INFO_UPDATE[] = "/navit/map_info_update";
        inline static constexpr const char SERVICE_SAVE_MAP[] = "/navit/map_info_save";
        inline static constexpr const char TOPIC_TASK_CONTROL[] = "/navit/task_control";

    private Q_SLOTS:
        /*@brief Draw a polygon
        */
        void drawPolygon(bool checked);
        /*@brief Draw a curve
        */
        void drawCurve(bool checked);
        /*@brief Draw a site
        */
        void drawSite(bool checked);
        /* @brief Save the polygon
        */
        void savePolygonButton();
        void saveCurveButton();
        void saveSiteButton();
        void pullMap();
        void pushMap();
        void saveMap();
        void clearMap();
        void selectAreaButton();
        void excuteAreaButton();
        void selectLineButton();
        void excuteLineButton();
        void selectPointButton();
        void excutePointButton();
        void executeReturnChargeButton();
        void generageGridMapButton();
        //for teb 示教建图
        void startRecordAreaIdButton();
        void saveAreaIdButton();
        void startTeachingPathButton();
        void saveTeachingPathButton();
        void saveNavigationPointButton();
        void loadKMLMapButton();
        void exportKMLMapButton();
        void executeAbortChargeButton();
    protected:  
        Ui::OperationToolsPanel *ui_;
        /*@brief Initialize the panel
        */
        void onInitialize();
        /* @brief Activate a tool by name
         * @param tool_class_id The class id of the tool to activate
        */
        void activateTool(const std::string& tool_class_id);
    private:
        void mapCallback(const std_msgs::String::ConstPtr& msg);
        
        template <typename T>
        void convertRosMessageToJson(const T& message, std::string& json_string);
        std::string convertProtobufToJson(const navit::protocol::map_info::MapInfo& map_info);
        void resetButtonAndBackToInteract();
        uint16_t convertStringToUint16(const std::string& str);
        rviz::VisualizationManager* context_;
        navit::protocol::map_info::ItemProperty set_property_;

        rviz::Tool* desired_tool_ = nullptr;
        ros::Publisher map_pub_, task_pub_;
        ros::ServiceClient save_map_client_, cmd_record_client_, call_grid_client_;
        ros::Publisher task_command_pub_;
        ros::Subscriber map_sub_;
        
        
        ros::NodeHandle nh_= ros::NodeHandle("/navit_rviz_panel");
        std::string map_save_path_ = "/home/yjh";
        std::string map_load_path_ = "/home/yjh";
        std::string kml_map_path_ = "/home/yjh/kml_info.kml";
        std::string kml_map_save_path_ = "/home/yjh/kml_info.kml";
    };
} // namespace navit_rviz_panel

#endif