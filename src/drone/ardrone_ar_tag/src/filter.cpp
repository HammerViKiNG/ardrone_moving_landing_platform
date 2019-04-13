#include <filter.h>


MAFilter::MAFilter(size_t window)
{
    values = new double[window];
    size = window;   
    this->window = 0;
    for (size_t i = 0; i < size; i++)
        values[i] = 0;
}


MAFilter::get_filtered_value(double value)
{
    double filtered_value;
    shift_values();
    values[size - 1] = value;
    window++;
    std::accumulate(values + size - window, values + size, filtered_value);
    return filtered_value;
}


MAFilter::shift_values(void)
{
    for (size_t i = 0; i < size - 1; i++)
        values[i] = values[i + 1];
}
