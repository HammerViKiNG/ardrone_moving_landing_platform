#include <numeric>
#include <cstddef>

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
