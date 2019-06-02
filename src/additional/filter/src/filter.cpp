#include "filter/filter.h"


MAFilter::MAFilter(size_t window) :
  values(window, 0),
  size(window)
{
}


double MAFilter::get_filtered_value(double new_value)
{
    values.push_back(new_value);
    return std::accumulate(values.begin(), values.end(), 0.0) / size;
}

