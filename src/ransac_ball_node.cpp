#include "ransac_ball_node.hpp"

RansacBallNode::RansacBallNode()
: rclcpp_lifecycle::LifecycleNode(std::string("ransac_ball_node"))
{
    /*ransac initialize begin*/
    ransac_ball = std::make_unique<RansacBall>(
        ball_r,
        max_loop,
        threshold,
        min_samples
    );
    /*ransac initialize end*/
}

int main()
{

}