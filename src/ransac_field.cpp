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

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<Eigen::Index> picked_cols;
    std::size_t k = 3;
    picked_cols.reserve(k);
    std::sample(indices.begin(), indices.end(),
                std::back_inserter(picked_cols),
                k,
                gen);
    
    Eigen::MatrixXd picked_points(scan_points.rows(), k);
    for (std::size_t i =0; i < k; i++) picked_points.col(i) = scan_points.col(picked_cols[i]);
}
