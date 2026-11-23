#include <gtest/gtest.h>
#include <cmath>
#include <memory>


#include "PathPlanner.hpp"
#include "FoxgloveServer.hpp"
#include "StateTracker.hpp"
#include "hytech_msgs.pb.h"
#include "MCAPLogger.hpp"


class PathPlannerTest: public ::testing::Test {

    protected:

    static void SetUpTestSuite() {
        core::MCAPLogger::create("/tmp/", mcap::McapWriterOptions(""), CONFIG_PATH);
        core::FoxgloveServer::create(CONFIG_PATH);
        core::StateTracker::create();
        


    }

    void set_car_pose(float x, float y, float yaw) {
        auto pose = std::make_shared<hytech_msgs::pose>();
        pose->mutable_position()->set_x(x);
        pose->mutable_position()->set_y(y);
        pose->mutable_orientation()->set_w(std::cos(yaw/2));
        pose->mutable_orientation()->set_z(std::sin(yaw/2));
        core::StateTracker::instance().handle_receive_protobuf_message(pose);

    }

    static void add_cone(dv_msgs::Cones& cones, float x, float y, dv_msgs::Cones_ConeColor color) {
        auto* cone = cones.add_cones();
        cone->mutable_position()->set_x(x);
        cone->mutable_position()->set_y(y);
        cone->set_color(color);

    }

};

// Test that there is no path with insufficient number of cones
TEST_F(PathPlannerTest, TooFewConesGivesEmptyPath) {
    set_car_pose(0.0f, 0.0f, 0.0f);
    dv_msgs:: Cones cones;
    add_cone(cones, 5.0f, 1.9f, dv_msgs::Cones_ConeColor_BLUE);
    add_cone(cones, 5.0f, -1.9f, dv_msgs::Cones_ConeColor_YELLOW);
    auto path = planning::plan_path(cones);
    EXPECT_TRUE(path.empty());
}