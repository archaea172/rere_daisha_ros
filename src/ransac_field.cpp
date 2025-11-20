#include "ransac_field.hpp"

RansacField::RansacField()
: pose_init_({0, 0, 0})
{

}

void RansacField::run(const Eigen::MatrixXd &scan_points)
{
    const Eigen::Index row_num = scan_points.rows();
    std::vector<Eigen::Index> indices(row_num);
    for (Eigen::Index i = 0; i < row_num; i++)
    {
        indices[i] = i;
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<Eigen::Index> picked_rows;
    std::size_t k = 2;
    picked_rows.reserve(k);
    std::sample(indices.begin(), indices.end(),
                std::back_inserter(picked_rows),
                k,
                gen);
    
    Eigen::MatrixXd picked_points(k, scan_points.cols());
    for (std::size_t i =0; i < k; i++) picked_points.row(i) = scan_points.row(picked_rows[i]);


}
