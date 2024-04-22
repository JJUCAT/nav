#include "navit_rviz_panel/rviz_panel.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(navit_rviz_panel::OperationToolsPanel, rviz::Panel)

namespace navit_rviz_panel {

OperationToolsPanel::OperationToolsPanel(QWidget * parent)
: rviz::Panel(parent), context_(nullptr)
{
    ui_ = new Ui::OperationToolsPanel;
    ui_->setupUi(this);

    QString polygonName = ui_->polygonNameLineEdit->text();
    QString curveName = ui_->curveNameLineEdit->text();
    QString siteName = ui_->siteNameLineEdit->text();

    // Allow the button to be checked and unchecked independently
    ui_->polygonButton->setAutoExclusive(false);
    ui_->curveButton->setAutoExclusive(false);
    ui_->siteButton->setAutoExclusive(false);

    ui_->polygonButton->setCheckable(true);
    ui_->curveButton->setCheckable(true);
    ui_->siteButton->setCheckable(true);

    // Connect signals and slots
    connect(ui_->polygonButton, SIGNAL(clicked(bool)), this, SLOT(drawPolygon(bool)));
    connect(ui_->curveButton, SIGNAL(clicked(bool)), this, SLOT(drawCurve(bool)));
    connect(ui_->siteButton, SIGNAL(clicked(bool)), this, SLOT(drawSite(bool)));
    connect(ui_->loadMapButton, SIGNAL(clicked()), this, SLOT(pullMap()));
    connect(ui_->pushMapButton, SIGNAL(clicked()), this, SLOT(pushMap()));
    connect(ui_->clearMapButton, SIGNAL(clicked()), this, SLOT(clearMap()));
    connect(ui_->saveMapButton, SIGNAL(clicked()), this, SLOT(saveMap()));

    connect(ui_->savePolygonButton, SIGNAL(clicked()), this, SLOT(savePolygonButton()));
    connect(ui_->saveCurveButton, SIGNAL(clicked()), this, SLOT(saveCurveButton()));
    connect(ui_->saveSiteButton, SIGNAL(clicked()), this, SLOT(saveSiteButton()));

    ui_->selectAreaButton->setAutoExclusive(false);
    ui_->excuteAreaButton->setAutoExclusive(false);
    ui_->selectLineButton->setAutoExclusive(false);
    ui_->excuteLineButton->setAutoExclusive(false);
    ui_->selectPointButton->setAutoExclusive(false);
    ui_->excutePointButton->setAutoExclusive(false);
    ui_->selectAreaButton->setCheckable(true);
    ui_->excuteAreaButton->setCheckable(false);
    ui_->selectLineButton->setCheckable(true);
    ui_->excuteLineButton->setCheckable(false);
    ui_->selectPointButton->setCheckable(true);
    ui_->excutePointButton->setCheckable(false);

    // First tab
    connect(ui_->generageGridMapButton, SIGNAL(clicked()), this, SLOT(generageGridMapButton()));
    // Second tab
    connect(ui_->loadKMLMapButton, SIGNAL(clicked()), this, SLOT(loadKMLMapButton()));
    connect(ui_->exportKMLMapButton, SIGNAL(clicked()), this, SLOT(exportKMLMapButton()));

    //Third tab 示教建图
    connect(ui_->startRecordAreaIdButton, SIGNAL(clicked()), this, SLOT(startRecordAreaIdButton()));
    connect(ui_->saveAreaIdButton, SIGNAL(clicked()), this, SLOT(saveAreaIdButton()));
    connect(ui_->startTeachingPathButton, SIGNAL(clicked()), this, SLOT(startTeachingPathButton()));
    connect(ui_->saveTeachingPathButton, SIGNAL(clicked()), this, SLOT(saveTeachingPathButton()));
    connect(ui_->saveNavigationPointButton, SIGNAL(clicked()), this, SLOT(saveNavigationPointButton()));

    // Fourth tab 任务
    connect(ui_->selectAreaButton, SIGNAL(clicked()), this, SLOT(selectAreaButton()));
    connect(ui_->excuteAreaButton, SIGNAL(clicked()), this, SLOT(excuteAreaButton()));
    connect(ui_->selectLineButton, SIGNAL(clicked()), this, SLOT(selectLineButton()));
    connect(ui_->excuteLineButton, SIGNAL(clicked()), this, SLOT(excuteLineButton()));
    connect(ui_->selectPointButton, SIGNAL(clicked()), this, SLOT(selectPointButton()));
    connect(ui_->excutePointButton, SIGNAL(clicked()), this, SLOT(excutePointButton()));
    connect(ui_->executeReturnChargeButton, SIGNAL(clicked()), this, SLOT(executeReturnChargeButton()));
    connect(ui_->executeAbortChargeButton, SIGNAL(clicked()), this, SLOT(executeAbortChargeButton()));



    // Fill combo boxes with types
    ui_->polygonTypeComboBox->addItems(QStringList() << "全覆盖类型" << "禁行区域" << "禁止掉头区域" << "减速区域" << "禁止绕障区域");
    ui_->curveTypeComboBox->addItems(QStringList() << "贝塞尔曲线" << "直线" << "圆弧" << "示教路线");
    ui_->siteTypeComboBox->addItems(QStringList() << "普通站点" << "工作点" << "充电点" << "充电点前置点");

    nh_.param<std::string>("map_save_path", map_save_path_, map_save_path_);
    map_pub_ = nh_.advertise<std_msgs::String>(TOPIC_MAP_INFO_UPDATE, 1);
    task_pub_ = nh_.advertise<std_msgs::String>(TOPIC_TASK_CONTROL, 1);
    save_map_client_ = nh_.serviceClient<std_srvs::Empty>(SERVICE_SAVE_MAP);
    task_command_pub_ = nh_.advertise<navit_msgs::TaskCommand>("/navit/task_command", 1);
    cmd_record_client_ = nh_.serviceClient<navit_msgs::CmdRecordGeometry>("/navit/cmd_record_geometry");
    call_grid_client_ = nh_.serviceClient<navit_msgs::BuildGridMap>("/navit/build_grid_map");
    map_sub_ = nh_.subscribe(TOPIC_MAP_INFO, 1, &OperationToolsPanel::mapCallback, this);

}
void OperationToolsPanel::onInitialize()
{
  rviz::Panel::onInitialize();
  context_ = vis_manager_;
}

void OperationToolsPanel::mapCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received map from map_server...");
    std::string json_string = msg->data;
    google::protobuf::util::JsonParseOptions options;
    options.ignore_unknown_fields = true;
    navit::protocol::map_info::MapInfo sub_map_info;
    google::protobuf::util::JsonStringToMessage(json_string, &sub_map_info, options);

    std::cout << "json_string = " << json_string << std::endl;
    ROS_INFO("Received map from map_server...");
}

void OperationToolsPanel::savePolygonButton() {
    ROS_INFO("Saving polygon...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            if (!polygonTool->savePolygon()) {
                ROS_ERROR("cannot save polygon.");
                return;
            }
        } else {
            ROS_ERROR("cannot cast to CombinedSelectionTool.");
            return;
        }
    }
    resetButtonAndBackToInteract();
    ui_->polygonButton->setChecked(false);
    ROS_INFO("Saved polygon...");
}

void OperationToolsPanel::saveCurveButton() {
    ROS_INFO("Saving Curve...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto lineTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            lineTool->saveLine();
        }
    }
    resetButtonAndBackToInteract();
    ui_->curveButton->setChecked(false);
    ROS_INFO("Saved Curve...");
}

void OperationToolsPanel::saveSiteButton() {
    ROS_INFO("Saving Curve...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto lineTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            lineTool->savePoint();
        }
    }
    resetButtonAndBackToInteract();
    ui_->siteButton->setChecked(false);
    ROS_INFO("Saved Curve...");
}

void OperationToolsPanel::drawPolygon(bool checked)
{
    static int64_t id = 0;
    QString objectName = ui_->polygonNameLineEdit->text();
    ROS_INFO_STREAM("Drawing polygon of name: " << objectName.toStdString());

    QString type = ui_->polygonTypeComboBox->currentText();
    ROS_INFO_STREAM("Drawing polygon of type: " << type.toStdString());

    set_property_.set_item_property(navit::protocol::map_info::ItemProperty::POLYGON_TOOL);
    if (checked) {
        if (type == "全覆盖类型") {
            set_property_.mutable_map_polygon()->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE);
        } else if (type == "禁行区域") {
            set_property_.mutable_map_polygon()->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_AREA);
        } else if (type == "禁止掉头区域") {
            set_property_.mutable_map_polygon()->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_TURN_AREA);
        } else if (type == "减速区域") {
            set_property_.mutable_map_polygon()->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_SLOW_DOWN_AREA);
        } else if (type == "禁止绕障区域") {
            set_property_.mutable_map_polygon()->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_BYPASS_OBS_AREA) ;
        }
        set_property_.mutable_map_polygon()->set_name(objectName.toStdString());
        set_property_.mutable_map_polygon()->set_id(id);
        id ++;
        activateTool("rviz_combined_tool/CombinedSelectionTool");
    } else {
        ui_->polygonButton->setChecked(false);
    }
}

void OperationToolsPanel::drawCurve(bool checked)
{
    static int64_t id = 0;
    QString objectName = ui_->curveNameLineEdit->text();
    ROS_INFO_STREAM("Drawing polygon of name: " << objectName.toStdString());

    QString type = ui_->curveTypeComboBox->currentText();
    ROS_INFO_STREAM("Drawing curve of type: " << type.toStdString());

    set_property_.set_item_property(navit::protocol::map_info::ItemProperty::LINE_TOOL);
    if (checked) {
        if (type == "贝塞尔曲线") {
            set_property_.mutable_map_line()->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_BEZIER);
        } else if (type == "直线") {
            set_property_.mutable_map_line()->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_DISCRETE);
        } else if (type == "圆弧") {
            set_property_.mutable_map_line()->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_ARC);
        } else if (type == "示教路线") {
            set_property_.mutable_map_line()->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING);
        } else {
            set_property_.mutable_map_line()->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_DEFAULT);
        }
        set_property_.mutable_map_line()->set_name(objectName.toStdString());
        set_property_.mutable_map_line()->set_id(id);
        id ++;
        activateTool("rviz_combined_tool/CombinedSelectionTool");
    } else {
        ui_->polygonButton->setChecked(false);
    }
}

void OperationToolsPanel::drawSite(bool checked)
{
   static int64_t id = 0;
    QString objectName = ui_->siteNameLineEdit->text();
    ROS_INFO_STREAM("Drawing polygon of name: " << objectName.toStdString());

    QString type = ui_->siteTypeComboBox->currentText();
    ROS_INFO_STREAM("Drawing curve of type: " << type.toStdString());

    set_property_.set_item_property(navit::protocol::map_info::ItemProperty::POINT_TOOL);
    if (checked) {
        if (type == "普通站点") {
            set_property_.mutable_map_point()->set_type(navit::protocol::map_info::MapPoint::POINT_TYPE_COMMON_STATION);
        } else if (type == "充电点") {
            set_property_.mutable_map_point()->set_type(navit::protocol::map_info::MapPoint::POINT_TYPE_CHARGE_PILE_STATION);
        } else if (type == "充电点前置点"){
            set_property_.mutable_map_point()->set_type(navit::protocol::map_info::MapPoint::POINT_TYPE_NEARBY_CHARGE_PILE_STATION);
        } else {
            set_property_.mutable_map_point()->set_type(navit::protocol::map_info::MapPoint::POINT_TYPE_DEFAULT);
        }
        set_property_.mutable_map_point()->set_name(objectName.toStdString());
        set_property_.mutable_map_point()->set_id(id);
        id ++;
        activateTool("rviz_combined_tool/CombinedSelectionTool");
    } else {
        ui_->polygonButton->setChecked(false);
    }
}

void OperationToolsPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

void OperationToolsPanel::load(const rviz::Config & config)
{
    rviz::Panel::load(config);
}

void OperationToolsPanel::activateTool(const std::string& tool_class_id)
{
    rviz::ToolManager* tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {
        if (tool_manager->getTool(i)->getClassId().toStdString() == tool_class_id) {
            desired_tool_ = tool_manager->getTool(i);
            break;
        }
    }
    if (desired_tool_ != nullptr) {
        ROS_INFO_STREAM("We found the draw tool " << tool_class_id << ", setting it as the current tool.");
        tool_manager->setCurrentTool(desired_tool_);
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto lineTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            lineTool->setItemProperty(set_property_);
        }
    }
}
void OperationToolsPanel::generageGridMapButton() {
    ROS_INFO("Pushing map for generate grid map...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    navit::protocol::map_info::MapInfo map_info;
    rviz::ToolManager* tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {
        if (tool_manager->getTool(i)->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
            desired_tool_ = tool_manager->getTool(i);
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->getMap(map_info);
            }
            ROS_INFO("Map info size: %d", map_info.map_areas_size());
        }
    }
    navit_msgs::BuildGridMap build_grid_map;
    bool has_coverage = false;

    geometry_msgs::Polygon coverage_polygon, forbidden_polygon;
    geometry_msgs::Point32 point;
    std::vector<geometry_msgs::Polygon> forbidden_polygon_v;

    // map info
    if (map_info.map_areas().empty()) {
        return;
    }
    // for (int j = 0; j < map_info.map_areas().size(); ++j) {
    int last_map_area_index = map_info.map_areas().size() - 1;
        build_grid_map.request.frame_id = "map";
        build_grid_map.request.label = map_info.map_areas(last_map_area_index).name();
        if (map_info.map_areas(last_map_area_index).type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
            for (int k = 0; k < map_info.map_areas(last_map_area_index).path_size(); ++k) {
                point.x = map_info.map_areas(last_map_area_index).path(k).x();
                point.y = map_info.map_areas(last_map_area_index).path(k).y();
                coverage_polygon.points.push_back(point);
            }
            has_coverage = true;
        } else if (map_info.map_areas(last_map_area_index).type() == navit::protocol::map_info::MapArea::AREA_TYPE_FORBIDDEN_AREA){
            for (int k = 0; k < map_info.map_areas(last_map_area_index).path_size(); ++k) {
                point.x = map_info.map_areas(last_map_area_index).path(k).x();
                point.y = map_info.map_areas(last_map_area_index).path(k).y();
                forbidden_polygon.points.push_back(point);
            }
            forbidden_polygon_v.push_back(forbidden_polygon);
        }
  //  }
    build_grid_map.request.polygons.push_back(coverage_polygon);

    for (int i = 0; i < forbidden_polygon_v.size(); ++i) {
        build_grid_map.request.polygons.push_back(forbidden_polygon_v[i]);
    }
    if (has_coverage) {
        ROS_INFO("Calling build grid map...");
        // 打印build grid map
        call_grid_client_.call(build_grid_map);
        has_coverage = false;
        build_grid_map.request.polygons.clear();
    }
    ROS_INFO("Pushed map for generate grid map...");

}
void OperationToolsPanel::pushMap() {
    ROS_INFO("Pushing map to map_server...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    navit::protocol::map_info::MapInfo map_info;
    rviz::ToolManager* tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {

        if (tool_manager->getTool(i)->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {

            desired_tool_ = tool_manager->getTool(i);
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->getMap(map_info);
            }
            ROS_INFO("Map info size: %d", map_info.map_areas_size());
        }
    }

    std_msgs::String map_info_msg;
    convertRosMessageToJson(map_info, map_info_msg.data);
    map_pub_.publish(map_info_msg);
}

void OperationToolsPanel::pullMap() {
    ROS_INFO("Load map from map path...");
    // find path
    QString load_map_path = ui_->loadPathEdit->text();
    // load json from file
    if (!load_map_path.isEmpty()) {
        map_load_path_ = load_map_path.toStdString();
    }
    std::ifstream in(map_load_path_ + "/map_info.json");
    navit::protocol::map_info::MapInfo sub_map_info;
    if (in) {
        std::string json_string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        google::protobuf::util::JsonParseOptions options;
        options.ignore_unknown_fields = true;

        google::protobuf::util::JsonStringToMessage(json_string, &sub_map_info, options);

        std::cout << "json_string = " << json_string << std::endl;
        ROS_INFO("Received map from map_server...");
    } else {
        ROS_ERROR("Failed to open file for reading.");
    }

    activateTool("rviz_combined_tool/CombinedSelectionTool");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->setMap(sub_map_info);
        }
    }
    ROS_INFO("Pulled map from map_server...");
}

void OperationToolsPanel::saveMap() {
    ROS_INFO("Saving map...");
    std_srvs::Empty save_map_srv;

    if (save_map_client_.call(save_map_srv)) {
       ROS_INFO("Save map successfully.");
    } else {
       ROS_ERROR("Failed to save map.");
    }
    //save map info to map_info.json
    navit::protocol::map_info::MapInfo map_info;
    rviz::ToolManager* tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {
        if (tool_manager->getTool(i)->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
            desired_tool_ = tool_manager->getTool(i);
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->getMap(map_info);
            }
            ROS_INFO("Map info size: %d", map_info.map_areas_size());
        }
    }
    // find path
    QString save_map_path = ui_->savePathEdit->text();
    // save json to file
    if (!save_map_path.isEmpty()) {
        map_save_path_ = save_map_path.toStdString();
    }

    std::string json_string = convertProtobufToJson(map_info);
    std::ofstream out(map_save_path_ + "/map_info.json");
    if (out) {
        out << json_string;
        out.close();
        ROS_INFO("Saved map info to %s", (map_save_path_ + "/map_info.json").c_str());
    } else {
        ROS_ERROR("Failed to open file for writing.");
    }
    ROS_INFO("Saved map...");
}
void OperationToolsPanel::clearMap() {
    ROS_INFO("Clearing map...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->clearMap();
        }
    }

    ROS_INFO("Cleared map...");
}

void OperationToolsPanel::selectAreaButton() {
    ROS_INFO("SelectAreaButton...");
    activateTool("rviz_combined_tool/CombinedSelectionTool");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->selectArea();
        }
    }
    ROS_INFO("Finished selectAreaButton...");
}

void OperationToolsPanel::excuteAreaButton() {
    ROS_INFO("ExcuteAreaButton...");

    navit_msgs::TaskCommand task_command;
    QString task_polygon_id = ui_->polygonIndexLineEdit->text();
    QString task_polygon_excute_num = ui_->areaExecuteCountLineEdit->text();

    task_command.cmd = navit_msgs::TaskCommand::CMD_POLYGON;

    QStringList polygon_ids = task_polygon_id.split(',');
    for (int i = 0; i < polygon_ids.size(); ++i) {
        task_command.seq.push_back(polygon_ids[i].toStdString());
    }
    if (polygon_ids.size() > 1) {
        task_command.cmd = navit_msgs::TaskCommand::CMD_MuiltPolygons_WITH_AUTODOCK;
    }
    task_command.repeat_times = convertStringToUint16(task_polygon_excute_num.toStdString());

    task_command_pub_.publish(task_command);
    std::vector<std::string> polygons_ids;
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->getSelectArea(polygons_ids);
        }
    }
    // navigation::TaskControl task_control;

    // navigation::CmdGoCoverageRequest* cmd_go_coverage_request = new navigation::CmdGoCoverageRequest();
    // for (const std::string& polygon_id : polygons_ids) {
    //     cmd_go_coverage_request->add_areas(polygon_id);
    // }

    // task_control.set_cmd("go_coverage");
    // task_control.set_allocated_cmd_param_go_coverage(cmd_go_coverage_request);

    // std_msgs::String task_areas_msg;
    // convertRosMessageToJson(task_control, task_areas_msg.data);
    // task_pub_.publish(task_areas_msg);

    ui_->selectAreaButton->setChecked(false);
    resetButtonAndBackToInteract();

    ROS_INFO("Finished excuteAreaButton...");
}

void OperationToolsPanel::selectLineButton() {
    ROS_INFO("SelectLineButton...");
    activateTool("rviz_combined_tool/CombinedSelectionTool");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->selectLine();
        }
    }
    ROS_INFO("Finished selectLineButton...");
}

void OperationToolsPanel::excuteLineButton() {
    ROS_INFO("ExcuteLineButton...");
    navit_msgs::TaskCommand task_command;
    QString task_path_id = ui_->pathIndexLineEdit->text();
    QString task_path_excute_num = ui_->pathExecuteCountLineEdit->text();
    task_command.cmd = navit_msgs::TaskCommand::CMD_PATH;

    QStringList path_ids = task_path_id.split(',');
    for (int i = 0; i < path_ids.size(); ++i) {
        task_command.seq.push_back(path_ids[i].toStdString());
    }

    task_command.repeat_times = convertStringToUint16(task_path_excute_num.toStdString());
    task_command_pub_.publish(task_command);

    std::vector<std::string> lines_ids;
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->getSelectLine(lines_ids);
        }
    }
    // navigation::TaskControl task_control;

    // navigation::CmdGoPathRequest* cmd_go_line_request = new navigation::CmdGoPathRequest();
    // for (const std::string& line_id : lines_ids) {
    //     cmd_go_line_request->add_stations(line_id);
    // }

    // task_control.set_cmd("go_line");
    // task_control.set_allocated_cmd_param_go_path(cmd_go_line_request);

    // std_msgs::String task_areas_msg;
    // convertRosMessageToJson(task_control, task_areas_msg.data);
    // task_pub_.publish(task_areas_msg);

    ui_->selectLineButton->setChecked(false);
    resetButtonAndBackToInteract();

    ROS_INFO("Finished excuteLineButton...");
}

template <typename T>
void OperationToolsPanel::convertRosMessageToJson(const T& message, std::string& json_string) {
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  options.always_print_primitive_fields = true;
  google::protobuf::util::MessageToJsonString(message, &json_string, options);
}

void OperationToolsPanel::resetButtonAndBackToInteract() {
    // TODO(czk)取消激活当前工具后，是否将当前工具设置为interact？，这样可以继续缩放rviz。
    if (context_) {
        rviz::ToolManager* tool_manager = context_->getToolManager();
        for (int i = 0; i < tool_manager->numTools(); ++i) {
            rviz::Tool* tool = tool_manager->getTool(i);
            if (tool && tool->getClassId() == "rviz/Interact") {
                tool_manager->setCurrentTool(tool);
                break;
            }
        }
    }
}
void OperationToolsPanel::selectPointButton() {
    ROS_INFO("SelectPointButton...");

    activateTool("rviz_combined_tool/CombinedSelectionTool");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->selectPoint();
        }
    }
    ROS_INFO("Finished selectPointButton...");
}
void OperationToolsPanel::excutePointButton() {
    navit_msgs::TaskCommand task_command;
    QString task_point_id = ui_->pointIndexLineEdit->text();
    QString task_point_excute_num = ui_->siteExecuteCountLineEdit->text();
    task_command.cmd = navit_msgs::TaskCommand::CMD_POINT;

    QStringList point_ids = task_point_id.split(',');
    for (int i = 0; i < point_ids.size(); ++i) {
        task_command.seq.push_back(point_ids[i].toStdString());
    }

    task_command.repeat_times = convertStringToUint16(task_point_excute_num.toStdString());
    task_command_pub_.publish(task_command);

    // ROS_INFO("ExcutePointButton...");
    // if (desired_tool_ == nullptr) {
    //     ROS_ERROR("No tool is activated.");
    //     return;
    // }
    // std::vector<navit::protocol::map_info::Point> point;
    // if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
    //     if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
    //         polygonTool->getSelectPoint(point);
    //     }
    // }
    // navigation::TaskControl task_control;

    // navigation::CmdGoPoseRequest* cmd_go_point_request = new navigation::CmdGoPoseRequest();
    // cmd_go_point_request->set_x(point[0].x());
    // cmd_go_point_request->set_y(point[0].y());

    // task_control.set_cmd("go_pose");
    // task_control.set_allocated_cmd_param_go_pose(cmd_go_point_request);

    // std_msgs::String task_areas_msg;
    // convertRosMessageToJson(task_control, task_areas_msg.data);
    // task_pub_.publish(task_areas_msg);

    // ui_->selectPointButton->setChecked(false);
    // resetButtonAndBackToInteract();

    ROS_INFO("Finished excutePointButton...");
}
void OperationToolsPanel::startRecordAreaIdButton() {
    QString coordinate_name = ui_->coordinateEdit->text();
    navit_msgs::CmdRecordGeometry cmd_record_geometry;
    cmd_record_geometry.request.frame_id = coordinate_name.toStdString();
    cmd_record_geometry.request.mode = 1;

    cmd_record_geometry.request.cmd = navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_POLYGON;
    ROS_INFO("Click start record area button.");
    if (cmd_record_client_.call(cmd_record_geometry)) {
        ROS_INFO("Start record polygon successfully.");
    } else {
        ROS_ERROR("Failed to start record polygon.");
    }
    //TODO(czk) : 增加弹窗提示
}
void OperationToolsPanel::saveAreaIdButton() {
    QString coordinate_name = ui_->coordinateSystemLabel->text();
    navit_msgs::CmdRecordGeometry cmd_record_geometry;
    cmd_record_geometry.request.frame_id = coordinate_name.toStdString();

    QString polygon_id = ui_->areaIdRecordLineEdit->text();
    cmd_record_geometry.request.cmd = navit_msgs::CmdRecordGeometryRequest::CMD_FINISH_RECORD;
    ROS_INFO("Click start save area button.");
    if (cmd_record_client_.call(cmd_record_geometry)) {
        ROS_INFO("Save record polygon successfully.");
        std::vector<geometry_msgs::PoseStamped> poses_stamped = cmd_record_geometry.response.poses_stamped;

        navit::protocol::map_info::MapInfo expand_polygon;
        navit::protocol::map_info::MapArea* map_area = expand_polygon.add_map_areas();
        //TODO(czk): 增加类型选择，没时间了，先写死
        map_area->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE);
        map_area->set_name(polygon_id.toStdString());
        for (const geometry_msgs::PoseStamped& pose_stamped : poses_stamped) {
            navit::protocol::map_info::Point* map_point = map_area->add_path();
            map_point->set_x(pose_stamped.pose.position.x);
            map_point->set_y(pose_stamped.pose.position.y);
        }
        ROS_INFO("Record polygon successfully.");
        activateTool("rviz_combined_tool/CombinedSelectionTool");
        if (desired_tool_ == nullptr) {
            ROS_ERROR("No tool is activated.");
            return;
        }
        if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->expandMapPolygon(expand_polygon);
            }
        }

    } else {
        ROS_ERROR("Failed to save record polygon.");
    }
    resetButtonAndBackToInteract();
}
void OperationToolsPanel::startTeachingPathButton() {
    QString coordinate_name = ui_->coordinateEdit->text();
    navit_msgs::CmdRecordGeometry cmd_record_geometry;
    cmd_record_geometry.request.frame_id = coordinate_name.toStdString();
    cmd_record_geometry.request.mode = 1;
    cmd_record_geometry.request.cmd = navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_PATH;
    ROS_INFO("Click start save path button.");
    if (cmd_record_client_.call(cmd_record_geometry)) {
        ROS_INFO("Start record path successfully.");
    } else {
        ROS_ERROR("Failed to start record path.");
    }
}

void OperationToolsPanel::saveTeachingPathButton() {
    QString coordinate_name = ui_->coordinateSystemLabel->text();
    navit_msgs::CmdRecordGeometry cmd_record_geometry;
    cmd_record_geometry.request.frame_id = coordinate_name.toStdString();
    QString path_id = ui_->teachingPathRecordLineEdit->text();
    cmd_record_geometry.request.cmd = navit_msgs::CmdRecordGeometryRequest::CMD_FINISH_RECORD;
    if (cmd_record_client_.call(cmd_record_geometry)) {
        ROS_INFO("Start record path successfully.");
         std::vector<geometry_msgs::PoseStamped> poses_stamped = cmd_record_geometry.response.poses_stamped;
        navit::protocol::map_info::MapInfo expand_line;
        navit::protocol::map_info::MapLine* map_path = expand_line.add_map_lines();

        map_path->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING);
        map_path->set_name(path_id.toStdString());

        for (const geometry_msgs::PoseStamped& pose_stamped : poses_stamped) {
            navit::protocol::map_info::Point* map_point = map_path->add_path();
            map_point->set_x(pose_stamped.pose.position.x);
            map_point->set_y(pose_stamped.pose.position.y);
        }

        activateTool("rviz_combined_tool/CombinedSelectionTool");
        if (desired_tool_ == nullptr) {
            ROS_ERROR("No tool is activated.");
            return;
        }
        if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->expandMapLine(expand_line);
            }
        }
    } else {
        ROS_ERROR("Failed to start record path.");
    }
    resetButtonAndBackToInteract();
}

void OperationToolsPanel::saveNavigationPointButton() {

}

void OperationToolsPanel::executeReturnChargeButton() {
    navit_msgs::TaskCommand task_command;
    QString task_charing_excute_nums = ui_->returnChargeExecuteCountLineEdit->text();
    task_command.cmd = navit_msgs::TaskCommand::CMD_CHARGE;
    task_command.repeat_times = convertStringToUint16(task_charing_excute_nums.toStdString());
    task_command_pub_.publish(task_command);

    ROS_INFO("Finished excuteReturnDockButton...");
}

void OperationToolsPanel::executeAbortChargeButton() {
    navit_msgs::TaskCommand task_command;
    QString task_charing_excute_nums = ui_->returnChargeExecuteCountLineEdit->text();
    task_command.cmd = navit_msgs::TaskCommand::CMD_ABORT_CHARGE;
    task_command.repeat_times = convertStringToUint16(task_charing_excute_nums.toStdString());
    task_command_pub_.publish(task_command);

    ROS_INFO("Finished executeAbortChargeButton...");
}

std::string OperationToolsPanel::convertProtobufToJson(const navit::protocol::map_info::MapInfo& map_info) {
    std::string json_string;
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;

    google::protobuf::util::MessageToJsonString(map_info, &json_string, options);

    return json_string;
}

uint16_t OperationToolsPanel::convertStringToUint16(const std::string& str) {
    try {
        unsigned long num = std::stoul(str);
        if (num > std::numeric_limits<uint16_t>::max()) {
            ROS_ERROR("Number out of range for uint16_t.");
            return 0;
        }
        uint16_t result = static_cast<uint16_t>(num);
        ROS_INFO("Result = %d", result);
        return result;
    } catch (const std::invalid_argument& e) {
        ROS_ERROR("Invalid argument. %s", e.what());
        return 0;
    } catch (const std::out_of_range& e) {
        ROS_ERROR("Number out of range. %s", e.what());
        return 0;
    }
}
void OperationToolsPanel::loadKMLMapButton() {
    ROS_INFO("Load kml map from map path...");
    // find path
    QString load_kml_map_path = ui_->loadKMLPathEdit->text();

    // load json from file
    if (!load_kml_map_path.isEmpty()) {
        kml_map_path_ = load_kml_map_path.toStdString();
    }
    std::ifstream in();
    std::vector<navit_common::coordinate_trans::NamedUTMPoint> named_utm_points;

    navit_common::coordinate_trans::parseKMLAndConvertToUtm(kml_map_path_, named_utm_points);
    navit_common::coordinate_trans::utm2map(named_utm_points);
    navit::protocol::map_info::MapInfo sub_map_info;
    //named_utm_points converter to sub_map_info
    for (const auto& named_utm_point : named_utm_points) {
        navit::protocol::map_info::MapArea* map_area = sub_map_info.add_map_areas();
        navit::protocol::map_info::MapLine* map_line = sub_map_info.add_map_lines();

        if (named_utm_point.type == navit_common::coordinate_trans::NamedUTMPoint::POLYGON) {
            map_area->set_name(named_utm_point.name);
            std::cout << "navit_common::coordinate_trans::NamedUTMPoint::POLYGON " << std::endl;
            map_area->set_type(navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE);
            for (const auto& utm_point : named_utm_point.utm_points) {
                navit::protocol::map_info::Point* map_point = map_area->add_path();
                map_point->set_x(utm_point.easting); //offset for visualize
                map_point->set_y(utm_point.northing);//offset for visualize
            }
        }

        if (named_utm_point.type == navit_common::coordinate_trans::NamedUTMPoint::LINE) {
            map_line->set_name(named_utm_point.name);
            std::cout << "navit_common::coordinate_trans::NamedUTMPoint::LINE " << std::endl;
            map_line->set_type(navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING);
            for (const auto& utm_point : named_utm_point.utm_points) {
                navit::protocol::map_info::Point* map_point = map_line->add_path();
                map_point->set_x(utm_point.easting); //offset for visualize
                map_point->set_y(utm_point.northing);//offset for visualize
            }
        }
    }

    activateTool("rviz_combined_tool/CombinedSelectionTool");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    if (desired_tool_->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
        if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
            polygonTool->setMap(sub_map_info);
        }
    }

    ROS_INFO("Pulled map from map_server...");
}
void OperationToolsPanel::exportKMLMapButton() {
    ROS_INFO("Saving map to Kml file...");
    if (desired_tool_ == nullptr) {
        ROS_ERROR("No tool is activated.");
        return;
    }
    QString export_kml_map_path = ui_->loadKMLPathEdit->text();

    if (!export_kml_map_path.isEmpty()) {
        kml_map_save_path_ = export_kml_map_path.toStdString();
    }
    navit::protocol::map_info::MapInfo map_info;
    rviz::ToolManager* tool_manager = context_->getToolManager();
    for (int i = 0; i < tool_manager->numTools(); ++i) {
        if (tool_manager->getTool(i)->getClassId().toStdString() == "rviz_combined_tool/CombinedSelectionTool") {
            desired_tool_ = tool_manager->getTool(i);
            if (auto polygonTool = dynamic_cast<rviz_combined_tool::CombinedSelectionTool*>(desired_tool_)) {
                polygonTool->getMap(map_info);
            }
            ROS_INFO("Map info size: %d", map_info.map_areas_size());
        }
    }
    if (map_info.map_areas().empty() && map_info.map_lines().empty() && map_info.map_points().empty()) {
        ROS_WARN("There is no map info. check your task map.");
        return;
    }

    std::vector<navit_common::coordinate_trans::NamedUTMPoint> named_utm_points;

    for (const auto& polygon : map_info.map_areas()) {
        navit_common::coordinate_trans::NamedUTMPoint named_utm_point;

        if (polygon.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
            named_utm_point.name = polygon.name();
            named_utm_point.type = navit_common::coordinate_trans::NamedUTMPoint::POLYGON;
            geodesy::UTMPoint utm_point;
            for (int i = 0; i < polygon.path_size(); ++i) {
                utm_point.easting = polygon.path(i).x();
                utm_point.northing = polygon.path(i).y();
                named_utm_point.utm_points.push_back(utm_point);
            }
            named_utm_points.push_back(named_utm_point);
        }
    }
    for (const auto& line : map_info.map_lines()) {
        navit_common::coordinate_trans::NamedUTMPoint named_utm_point;
        if (line.type() == navit::protocol::map_info::MapLine::LINE_TYPE_TEACHING) {
            named_utm_point.name = line.name();
            named_utm_point.type = navit_common::coordinate_trans::NamedUTMPoint::LINE;
            geodesy::UTMPoint utm_point;
            for (int i = 0; i < line.path_size(); ++i) {
                utm_point.easting = line.path(i).x();
                utm_point.northing = line.path(i).y();
                named_utm_point.utm_points.push_back(utm_point);
            }
            named_utm_points.push_back(named_utm_point);
        }
    }

    for (const auto& point : map_info.map_points()) {
        navit_common::coordinate_trans::NamedUTMPoint named_utm_point;
        if (point.type() == navit::protocol::map_info::MapPoint::POINT_TYPE_COMMON_STATION) {
            named_utm_point.name = point.name();
            named_utm_point.type = navit_common::coordinate_trans::NamedUTMPoint::LINE;
            geodesy::UTMPoint utm_point;
            utm_point.easting = point.point().x();
            utm_point.northing = point.point().y();
            named_utm_point.utm_points.push_back(utm_point);
        }
    }
    // navit_common::coordinate_trans::map2utm(named_utm_points);
    std::cout << "named_utm_points size is " << named_utm_points.size() << std::endl;
    std::cout << "named_utm_points size is " << named_utm_points.size() << std::endl;
    std::cout << "named_utm_points size is " << named_utm_points.size() << std::endl;

    navit_common::coordinate_trans::convertLocalToKml(named_utm_points, kml_map_save_path_);
    ROS_INFO("Saved map to Kml file.");
}
} // namespace navit_rviz_panel