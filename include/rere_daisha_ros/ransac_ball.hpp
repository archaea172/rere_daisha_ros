#include <Eigen/Dense>
#include <vector>

class RansacBall
{
public:
    RansacBall(float r);

private:
    std::vector<std::vector<float>> run();

    const float ball_r;
};