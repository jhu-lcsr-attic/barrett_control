#ifndef __BARD_COMMON_TIME_LORD_H
#define __BARD_COMMON_TIME_LORD_H

#include <boost/scoped_ptr.hpp>

namespace bard_common {
  class TimeLord {
  public:
    ros::Time gettime_ros() {

    }

  protected:
    TimeLord() {
      // Check if ROS
    }

    static boost::scoped_ptr<TimeLord> singleton_;
  };
}

#endif // ifndef __BARD_COMMON_TIME_LORD_H
