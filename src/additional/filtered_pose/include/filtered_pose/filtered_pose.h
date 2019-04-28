#ifndef FILTERED_POSE_H
#define FILTERED_POSE_H

#include "pose_rpy/pose_rpy.h"
#include "filter/filter.h"


class FilteredPose
{
    public:
        FilteredPose(size_t window);
        void filter_pose(const PoseRPY& pose);
        PoseRPY get_filtered_pose(void);

    private:
        PoseRPY filtered_pose;
        
        MAFilter* filters[6];
};

#endif
