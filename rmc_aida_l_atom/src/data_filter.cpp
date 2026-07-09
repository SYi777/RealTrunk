#include "data_filter.h"
#include <string.h>


float DataFilter::Refilter(uint8_t joint_id, float in_data, uint8_t in_count)
{
    int j = 0;
    float re_average_data = 0;

    filter_data_buf_k_[joint_id]++;

    if (filter_data_buf_k_[joint_id] > in_count)
    {
        filter_data_buf_k_[joint_id] = in_count;

        filter_data_buf_sum_[joint_id] = filter_data_buf_sum_[joint_id] - filter_data_buf_[joint_id][0];

        for (j = 1; j < in_count; j++)
        {
            filter_data_buf_[joint_id][j - 1] = filter_data_buf_[joint_id][j];
        }
        filter_data_buf_[joint_id][in_count - 1] = in_data;

        filter_data_buf_sum_[joint_id] += in_data;

        re_average_data = filter_data_buf_sum_[joint_id] / in_count;
    }
    else
    {
        filter_data_buf_[joint_id][filter_data_buf_i_[joint_id]] = in_data;

        filter_data_buf_i_[joint_id]++;

        filter_data_buf_sum_[joint_id] += in_data;

        re_average_data = filter_data_buf_sum_[joint_id] / filter_data_buf_i_[joint_id];
    }

    return re_average_data;
}

void DataFilter::ResetFilterData()
{
    memset(filter_data_buf_sum_, 0, sizeof(filter_data_buf_sum_));
    memset(filter_data_buf_i_, 0, sizeof(filter_data_buf_i_));
    memset(filter_data_buf_k_, 0, sizeof(filter_data_buf_k_));
}
