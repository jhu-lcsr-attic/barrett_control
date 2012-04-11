/*
 * Copyright (c) 2012, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Johns Hopkins University. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BARD_HARDWARE_WAM_H
#define __BARD_HARDWARE_WAM_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <urdf/model.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/JointState.h>

#include <leoCAN/RTSocketCAN.h>
#include <barrett_direct/WAM.h>

#include <bard_common/util.h>
#include <bard_hardware/wam_interface.h>

namespace bard_hardware {
  class WAM : public WAMInterface
  {
    // RTT Properties
    std::string can_dev_name_;
    // see bard_hardware::WAMInterface

    // RTT Ports
    // see bard_hardware::WAMInterface

    // RTT Operations
    void calibrate_position(std::vector<double> &actual_positions);
    void set_velocity_warn(unsigned int thresh);
    void set_velocity_fault(unsigned int thresh);
    void set_torque_warn(unsigned int thresh);
    void set_torque_fault(unsigned int thresh);

    // See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    // See: http://www.orocos.org/forum/orocos/orocos-users/some-info-eigen-and-orocos
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:
    WAM(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook(); 
    void stopHook();
    void cleanupHook();

  private:
    // Free pointers to robot and canbus drivers
    void cleanup_internal();

    // Hardware hooks
    boost::scoped_ptr<leoCAN::RTSocketCAN> canbus_;
    boost::scoped_ptr<barrett_direct::WAM> robot_;

    // Other members
    bool needs_calibration_;
  };
}


#endif // ifndef __BARD_HARDWARE_WAM_H
