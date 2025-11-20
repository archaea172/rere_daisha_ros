#include "ransac_field.hpp"

RansacField::RansacField()
: pose_init_({0, 0, 0})
{

}

void RansacField::run(const Eigen::MatrixXd &scan_points)
{
    const Eigen::Index row_num = scan_points.cols();
    std::vector<Eigen::Index> indices(row_num);
    for (Eigen::Index i = 0; i < row_num; i++)
    {
        indices[i] = i;
    }
}
