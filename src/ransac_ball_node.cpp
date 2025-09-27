#include "ransac_ball_node.hpp"

RansacBallNode::RansacBallNode()
: rclcpp_lifecycle::LifecycleNode(std::string("ransac_ball_node"))
{
    /*ransac initialize begin*/
    float ball_r = 0.08;
    int max_loop = 100;
    float threshold = 0.02;
    int min_samples = 50;
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