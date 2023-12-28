#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"

#include "geometry_msgs/msg/twist.hpp"

class TestIntegrationTest : public::testing::Test
{
    protected :
        void SetUp() override
        {
            rclcpp::init(0, nullptr);
            test_node = rclcpp::Node::make_shared("test_integration_node");
        }

        void TearDown() override
        {
            rclcpp::shutdown();
        }

        void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            if(msg)
            {
                cmd_msg_count += 1;
            }
        }
        
        std::shared_ptr<rclcpp::Node> test_node;
        static std::string node_name;
        int cmd_msg_count = 0;
        int odom_msg_count = 0;

    public:
        // Function to set the node-name argument
        static void setNodeName(const std::string& name)
        {   
            node_name = name;
        }        
};

// Test if a ROS 2 Node is active or not
TEST_F(TestIntegrationTest, TestNodesActive)
{   
    RCLCPP_INFO(test_node->get_logger(), "Node Name Received : %s", node_name.c_str());

    // Wait for 5 seconds before getting the list of active nodes
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // List of active nodes
    auto active_nodes = test_node->get_node_names();
    
    // Find if a particular node exists in this list
    // std::find returns the iterator to the first occurence of the specified element
    // If the element is not found, an iterator to the end is returned.
    // Read more about this here : https://www.geeksforgeeks.org/std-find-in-cpp/
    bool is_active_node = std::find(active_nodes.begin(), active_nodes.end(), node_name) != active_nodes.end();

    ASSERT_TRUE(is_active_node)<< "Node '" << node_name << "' not found in the list of active nodes";
}

// Test if a ROS 2 Topic is active or not
TEST_F(TestIntegrationTest, TestTopicsActive)
{   
    // Get the list of active ROS 2 topics using node graph
    auto node_graph = test_node->get_node_graph_interface();
    auto active_topics = node_graph->get_topic_names_and_types();

    // Find if a particular topic exists in this list
    bool is_active_cmd_vel = std::find_if(active_topics.begin(), active_topics.end(),[this](const auto &topic)
    {
        return topic.first == "/bcr_bot/cmd_vel";
    }) != active_topics.end();
    
    bool is_active_odom = std::find_if(active_topics.begin(), active_topics.end(),[this](const auto &topic)
    {
        return topic.first == "/bcr_bot/odom";
    }) != active_topics.end();

    ASSERT_TRUE(is_active_cmd_vel) << "Topic '" << "/bcr_bot/cmd_vel" << "' not found in the list of active topics";
    ASSERT_TRUE(is_active_odom) << "Topic '" << "/bcr_bot/odom" << "' not found in the list of active topics";
}

// Check the rates of topics
TEST_F(TestIntegrationTest, TestTopicsRate)
{   
    // Subscriber to 'bcr_bot/cmd_vel' topic
    auto cmd_vel_sub = test_node->create_subscription<geometry_msgs::msg::Twist>("/bcr_bot/cmd_vel", 1, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            CmdCallback(msg);
        });
    
    // Spin the test_node to receive messages
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node);
    executor.spin();

    // Make sure the no. of messages received is in a specified range
    bool isInRange = (cmd_msg_count >= 180 && cmd_msg_count <= 220);

    RCLCPP_INFO(test_node->get_logger(), "Message count : %d", cmd_msg_count);

    ASSERT_TRUE(isInRange);  
}

std::string TestIntegrationTest::node_name = "line_follower";

int main(int argc, char **argv)
{   
    testing::InitGoogleTest(&argc, argv);

    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--node-name" && i + 1 < argc)
        {
            // Set the node_name argument
            TestIntegrationTest::setNodeName(argv[i + 1]);
        }
    }
    int result = RUN_ALL_TESTS();

    return result;
}
