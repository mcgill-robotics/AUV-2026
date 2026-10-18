#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "controls/attitude_controller.hpp"

class AttitudeControllerTest : public ::testing::Test 
{
    protected:
        static void SetUpTestSuite() 
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite() 
        {
            rclcpp::shutdown();
        }

        void SetUp() override
        {
            attitudeController = std::make_shared<controls::AttitudeController>();
        }

        void TearDown() override
        {
            attitudeController.reset();
        }

        std::shared_ptr<controls::AttitudeController> attitudeController;
};

// Verify Node exists
TEST_F(AttitudeControllerTest, NodeExists) 
{
    ASSERT_NE(attitudeController, nullptr);
    EXPECT_STREQ(attitudeController->get_name(), "attitude_controller");
    EXPECT_NE(attitudeController->pub_effort_, nullptr);
}