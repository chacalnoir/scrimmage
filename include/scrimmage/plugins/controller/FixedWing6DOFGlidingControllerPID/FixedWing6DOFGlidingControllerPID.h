/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFGLIDINGCONTROLLERPID_FIXEDWING6DOFGLIDINGCONTROLLERPID_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFGLIDINGCONTROLLERPID_FIXEDWING6DOFGLIDINGCONTROLLERPID_H_

#include <scrimmage/motion/Controller.h>
#include <scrimmage/common/PID.h>
#include <scrimmage/math/State.h>

#include <map>
#include <string>

namespace scrimmage {
namespace controller {

class FixedWing6DOFGlidingControllerPID : public scrimmage::Controller {
 public:
    FixedWing6DOFGlidingControllerPID();
    void init(std::map<std::string, std::string> &params) override;
    bool step(double t, double dt) override;

 protected:
    PID pitch_rate_pid_;
    PID roll_rate_pid_;
    scrimmage::Quaternion quat_body_;
    Eigen::Quaterniond rot_180_x_axis_;

    uint8_t input_altitude_rate_idx_ = 0;
    uint8_t input_turn_rate_idx_ = 0;

    int elevator_idx_ = 0;
    int aileron_idx_ = 0;
    int rudder_idx_ = 0;
    int throttle_idx_ = 0;
};
}  // namespace controller
}  // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFGLIDINGCONTROLLERPID_FIXEDWING6DOFGLIDINGCONTROLLERPID_H_
