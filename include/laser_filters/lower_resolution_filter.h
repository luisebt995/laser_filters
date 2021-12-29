/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  lower_resolution_filter.h
 *
 *  author: Francesc Folch Company <frafolcm@gmail.com>
 */

#ifndef LOWER_RESOLUTION_FILTER_H
#define LOWER_RESOLUTION_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters{

/** \brief A class to provide median filtering of laser scans in time*/
class LowerResolutionFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{

  public:
    float reduce_factor_;
    bool configure()
    {
      reduce_factor_ = 1.0;
      double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
      if(!getParam("reduce_factor", temp_replacement_value)){
        ROS_ERROR("Cannot configure LowerResolutionFilter: Didn't find 'reduce_factor' paramerer.");
        return false;
      }
      reduce_factor_= static_cast<float>(temp_replacement_value);
      ROS_INFO("CONFIGURED");
      return true;
    }

    virtual ~LowerResolutionFilter(){}

    /** \brief Update the filter and get the response
     * \param scan_in The new scan to filter
     * \param scan_out The filtered scan
     */
    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
    {
      //int reduce_factor=10

      ROS_INFO("Updating");
      if (!this->configured_)
      {
        ROS_ERROR("LaserArrayFilter not configured");
        return false;
      }

      ROS_INFO("Updating2");
      scan_out = scan_in;

      int size_array=std::ceil( (scan_in.angle_max - scan_in.angle_min) / scan_in.angle_increment );
      int size_new_array=size_array;
      while (size_new_array%(int)reduce_factor_!=0) {
        size_new_array=size_new_array-1;
      }
      scan_out.angle_increment=(scan_out.angle_increment*reduce_factor_);

      scan_out.ranges.resize(0);
      scan_out.intensities.resize(0);
      int j = 0;
      for (int i = 0; i < size_array; i += reduce_factor_) {
        scan_out.ranges.push_back(scan_in.ranges[i]);
        scan_out.intensities.push_back(scan_in.intensities[i]);
        ++j;
      }

//      scan_out.ranges.resize(size_new_array);
//      scan_out.ranges = ranges_out;

//      scan_out.intensities.resize(size_new_array);
//      scan_out.intensities = intensities_out;
//
//      int original_length = (scan_in.angle_max - scan_in.angle_min + scan_in.angle_increment) / scan_in.angle_increment;
//      int reduced_length = std::ceil( original_length / reduce_factor_);
//
//      // as we round up when defining reduced length, we have to take in count the last element of the array
//      // if the reduce factor is divisible by original length, we pick the nth last element (n=reduce_factor)
//      // else, nth last element will be defined by the mod operation
//      int mod_angles = original_length % reduce_factor_;
//      float angle_max_out;
//      int last_element;
//      if(mod_angles != 0) {
//        angle_max_out = scan_in.angle_max - scan_in.angle_increment*(mod_angles - 1);
//        last_element = original_length-mod_angles+1;
//      }
//      else {
//        angle_max_out = scan_in.angle_max - scan_in.angle_increment*(reduce_factor_ - 1);
//        last_element = original_length-reduce_factor_+1;
//      }
//
//
//      float angle_inc_out = scan_in.angle_increment * reduce_factor_;
//
//      std::vector<float> ranges_out;
//      std::vector<float> intensities_out;
//
//      int j = 0;
//      for (int i = 0; i <= last_element; i += reduce_factor_) {
//        ranges_out[j] = scan_in.ranges[i];
//        intensities_out[j] = scan_in.intensities[i];
//        ++j;
//      }
//
//      scan_out.angle_max = angle_max_out;
//
//      scan_out.ranges.resize(reduced_length);
//      scan_out.ranges = ranges_out;
//
//      scan_out.intensities.resize(reduced_length);
//      scan_out.intensities = intensities_out;
//
//      scan_out.angle_increment = angle_inc_out;
//
      ROS_INFO("Filtered out %f points from the laser scan", reduce_factor_);

      return true;
    }

};
};
#endif
