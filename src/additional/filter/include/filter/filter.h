#ifndef FILTER_H
#define FILTER_H

#include <numeric>
#include <cstddef>
#include <cstring>

class MAFilter
{
    public:
        MAFilter(size_t window);
        double get_filtered_value(double new_value);

    private:
        void shift_values(void);
        
        double value;
        double* values;
        size_t window, size;
};


#endif
