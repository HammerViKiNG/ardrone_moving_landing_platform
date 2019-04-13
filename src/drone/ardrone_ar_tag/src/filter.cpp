#include "ardrone_ar_tag/filter.h"


MAFilter::MAFilter(size_t window)
{
    values = new double[window];
    size = window;   
    this->window = 0;
    for (size_t i = 0; i < size; i++)
        values[i] = 0;
}


double MAFilter::get_filtered_value(double new_value)
{
    shift_values();
    values[size - 1] = new_value;
    window++;
    std::accumulate(values + size - window, values + size, value);
    return value;
}


void MAFilter::shift_values(void)
{
    for (size_t i = 0; i < size - 1; i++)
        values[i] = values[i + 1];
}
