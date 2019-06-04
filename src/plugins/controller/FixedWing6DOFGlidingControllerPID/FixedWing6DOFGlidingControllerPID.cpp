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

#include <scrimmage/plugins/controller/FixedWing6DOFGlidingControllerPID/FixedWing6DOFGlidingControllerPID.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Controller,
                scrimmage::controller::FixedWing6DOFGlidingControllerPID,
                FixedWing6DOFGlidingControllerPID_plugin)

namespace scrimmage {
namespace controller {

FixedWing6DOFGlidingControllerPID::FixedWing6DOFGlidingControllerPID() {
}

void FixedWing6DOFGlidingControllerPID::init(std::map<std::string, std::string> &params) {
    if (!sc::set_pid_gains(heading_rate_pid_, params["heading_rate_pid"], true)) {
        std::cout << "Failed to set FixedWing6DOFGlidingControllerPID heading rate gains" << std::endl;
    }

    if (!sc::set_pid_gains(altitude_rate_pid_, params["altitude_rate_pid"], true)) {
        std::cout << "Failed to set FixedWing6DOFGlidingControllerPID altitude rate gains" << std::endl;
    }

    // Inputs
    input_altitude_rate_idx_ = vars_.declare(VariableIO::Type::desired_altitude_rate, VariableIO::Direction::In);
    input_turn_rate_idx_ = vars_.declare(VariableIO::Type::desired_turn_rate, VariableIO::Direction::In);

    // Outputs
    elevator_idx_ = vars_.declare(VariableIO::Type::elevator, VariableIO::Direction::Out);
    aileron_idx_ = vars_.declare(VariableIO::Type::aileron, VariableIO::Direction::Out);
    rudder_idx_ = vars_.declare(VariableIO::Type::rudder, VariableIO::Direction::Out);
    throttle_idx_ = vars_.declare(VariableIO::Type::throttle, VariableIO::Direction::Out);

    // Set defaults
    vars_.output(elevator_idx_, 0.0);
    vars_.output(aileron_idx_, 0.0);
    vars_.output(rudder_idx_, 0.0);
    vars_.output(throttle_idx_, 0.0);
}

bool FixedWing6DOFGlidingControllerPID::step(double t, double dt) {
    heading_rate_pid_.set_setpoint(vars_.input(input_turn_rate_idx_));
    // Technically using yaw rate from the state instead of heading rate, but should work for this.
    double u_heading = heading_rate_pid_.step(dt, state_->ang_vel()(3));

    altitude_rate_pid_.set_setpoint(vars_.input(input_altitude_rate_idx_));
    double u_alt_rate = altitude_rate_pid_.step(dt, state_->vel()(2));

    std::cout << "Elevator Command: " << u_alt_rate << ", Aileron Command: " <<
        u_heading << ", Rudder Command: " << 0.0 << ", Throttle Command: " << 0.0 << std::endl;

    // Output the controls
    vars_.output(elevator_idx_, u_alt_rate);
    vars_.output(aileron_idx_, u_heading);
    // TODO: This should be driving sideslip to zero for a coordinated turn
    vars_.output(rudder_idx_, 0.0);
    vars_.output(throttle_idx_, 0.0);  // Always 0 for a gliding vehicle

    return true;
}
}  // namespace controller
}  // namespace scrimmage
