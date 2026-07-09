#ifndef DATAFILTER_H
#define DATAFILTER_H

#include <cstdint>


class DataFilter
{
private:
    float filter_data_buf_[8][200] = {0};
    float filter_data_buf_sum_[8] = {0};
    int filter_data_buf_i_[8] = {0};
    int filter_data_buf_k_[8] = {0};

public:
    /// @brief 滑动滤波函数
    /// @param joint_id 关节号
    /// @param in_data 每次的输入数据
    /// @param in_count 滤波深度
    /// @return 滤波后数据
    float Refilter(uint8_t joint_id, float in_data, uint8_t in_count);

    void ResetFilterData();
};

#endif DATAFILTER_H