#include "mppi.hpp"

MPPIControler::MPPIControler(int sample_num, int dim_num)
: sample_num_(sample_num), dim_num_(dim_num), gen(1234), uni(0.0, 1.0)
{
    
}

Eigen::MatrixXd MPPIControler::sampling()
{

}