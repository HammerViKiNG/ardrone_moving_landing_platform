#include "filter.h"
#include <ros/ros.h>


MAFilter::MAFilter(size_t window)
{
    values = new double[window];
    size = window;   
    this->window = 0;
    memset(values, 0, size);
}


double MAFilter::get_filtered_value(double new_value)
{
    shift_values();
    values[size-1] = new_value;
    if (window < size)
        window++;
    //ROS_INFO_STREAM(values[size-1]);
    value = 0;
    for (size_t i = size - window; i < size; i++)
        value += values[i];
    value /= window;
    return value;
}


void MAFilter::shift_values(void)
{
    for (size_t i = 0; i < size - 1; i++)
        values[i] = values[i + 1];
}
