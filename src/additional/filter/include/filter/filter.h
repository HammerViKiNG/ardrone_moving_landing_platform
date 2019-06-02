#ifndef FILTER_H
#define FILTER_H

#include <numeric>
#include <cstddef>
#include <cstring>
#include <boost/circular_buffer.hpp>

class MAFilter
{
    public:
        MAFilter(size_t window);
        double get_filtered_value(double new_value);

    private:
        void shift_values(void);
        
        boost::circular_buffer<double> values;
        size_t size;
};


#endif
