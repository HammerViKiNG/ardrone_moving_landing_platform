#include <numeric>

class MAFilter
{
    public:
        MAFilter(size_t window);
        get_filtered_value(double value);

    private:
        shift_values(void);
        
        double* values;
        size_t window, size;
};
