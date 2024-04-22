#include <navit_bt_navigator/simple_navigator_action_server_node.hpp>

using namespace BT;
using namespace navit_bt_nodes;


void SimpleNavigatorActionServer::executeCB(const navit_msgs::SimpleNavigatorGoalConstPtr &goal)
{
    std::string tree_file;
    tree_file = getParam<std::string>("tree_file", "simple_navigator_with_autodock_recovery_subtree.xml");

    std::string tree_full_path = ros::package::getPath("navit_bt_navigator") + "/behavior_trees/" + tree_file;
    ROS_INFO("tree_full_path: [%s]", tree_full_path.c_str());

    ros::NodeHandle bt_node;
    double server_timeout;
    double bt_loop_duration;

    BT::Blackboard::Ptr blackboard;
    blackboard = BT::Blackboard::create();

    // // Put items on the blackboard
    blackboard->set<ros::NodeHandle>("node", bt_node);  // NOLINT
    blackboard->set<double>("server_timeout", server_timeout);  // NOLINT
    blackboard->set<double>("bt_loop_duration", bt_loop_duration);  // NOLINT

    ROS_WARN("Plugins Loading.....");
    auto bt_ = std::make_unique<navit_bt_nodes::BehaviorTreeEngine>(plugin_lib_names_);
    ROS_WARN("Plugins Loading Finished.");
    auto tree = bt_->createTreeFromFile(tree_full_path, blackboard);

    StdCoutLogger logger_cout(tree);
    std::string logger_full_path_ = ros::package::getPath("navit_bt_navigator") + "/logger/" ;
    FileLogger logger_file(tree, (logger_full_path_+"simple_navigator.fbl").c_str());
    MinitraceLogger logger_minitrace(tree, (logger_full_path_+"simple_navigator.json").c_str());

    #ifdef ZMQ_FOUND
    PublisherZMQ publisher_zmq(tree);
    #endif
    printTreeRecursively(tree.rootNode());

    // Keep on ticking until you get either a SUCCESS or FAILURE state
    NodeStatus future_status = NodeStatus::RUNNING;
    bool start_cmd = goal->start;
    while (ros::ok() && future_status == NodeStatus::RUNNING && start_cmd)
    {
        future_status = tree.tickRoot();
        ros::Duration(0.001).sleep();

        if (future_status == NodeStatus::FAILURE)
            as_.setAborted(navit_msgs::SimpleNavigatorResult(), "Aborting on the goal because the BT node future_status is FAILURE");
        else if (future_status == NodeStatus::SUCCESS)
            as_.setSucceeded(navit_msgs::SimpleNavigatorResult(), "Goal reached.");

        if(as_.isPreemptRequested()){
            if(as_.isNewGoalAvailable()){
                auto new_goal = as_.acceptNewGoal();
                start_cmd = new_goal->start;
            }
            else{
                //notify the ActionServer that we've successfully preempted
                ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                as_.setPreempted();
            }
        }
    }

    // std::future<NodeStatus>  future_status = std::async(std::launch::async, [&]()->NodeStatus{ return tree.tickRoot(); } );
    // ROS_WARN("async BT tick start");

    // std::future_status async_status;
    // bool start_cmd = goal->start;
    // while (future_status.get() == NodeStatus::RUNNING && start_cmd)
    // {
    //     ROS_WARN("BT tick running");
    //     do {
    //         ROS_WARN("BT async wait in LOOP");
    //         async_status = future_status.wait_for(std::chrono::seconds(1));
    //         if (async_status == std::future_status::deferred) {
    //             std::cout << "deferred\n";
    //             ROS_WARN("BT deferred");
    //         } else if (async_status == std::future_status::timeout) {
    //             std::cout << "timeout\n";
    //             ROS_WARN("BT timeout");
    //         } else if (async_status == std::future_status::ready) {
    //             std::cout << "ready!\n";
    //             ROS_WARN("BT ready");
    //         }
    //         ROS_WARN("BT async LOOP wait end in a flow ");
    //     } while (async_status != std::future_status::ready);

    //     ROS_WARN("BT tick finish");

    //     if (future_status.get() == NodeStatus::FAILURE)
    //         as_.setAborted(navit_msgs::SimpleNavigatorResult(), "Aborting on the goal because the BT node status is FAILURE");
    //     else if (future_status.get() == NodeStatus::SUCCESS)
    //         as_.setSucceeded(navit_msgs::SimpleNavigatorResult(), "Goal reached.");

    //     if(as_.isPreemptRequested()){
    //         if(as_.isNewGoalAvailable()){
    //             auto new_goal = as_.acceptNewGoal();
    //             start_cmd = new_goal->start;
    //         }
    //         else{
    //             //notify the ActionServer that we've successfully preempted
    //             ROS_DEBUG_NAMED("simple_navigator_action_server_node","Server preempting the current goal");
    //             as_.setPreempted();
    //         }
    //     }
    // }

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_navigator");

    SimpleNavigatorActionServer simple_navigator("simple_navigator");
    ros::spin();

    return 0;
}