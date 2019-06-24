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
#include <scrimmage/math/Angles.h>
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
    // For later rotations
    Eigen::AngleAxisd aa(M_PI, Eigen::Vector3d::UnitX());
    rot_180_x_axis_ = Eigen::Quaterniond(aa);
}

void FixedWing6DOFGlidingControllerPID::init(std::map<std::string, std::string> &params) {
    if (!sc::set_pid_gains(pitch_rate_pid_, params["pitch_rate_pid"], true)) {
        std::cout << "Failed to set FixedWing6DOFGlidingControllerPID pitch rate gains" << std::endl;
    }

    if (!sc::set_pid_gains(roll_rate_pid_, params["roll_rate_pid"], true)) {
        std::cout << "Failed to set FixedWing6DOFGlidingControllerPID roll rate gains" << std::endl;
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
    // Update the quaternion
    quat_body_ = rot_180_x_axis_ * state_->quat();
    quat_body_.set(sc::Angles::angle_pi(quat_body_.roll()+M_PI),
                   quat_body_.pitch(), quat_body_.yaw());
    quat_body_.normalize();

    // Calculate the tracking errors
    double heading_rate_error = vars_.input(input_turn_rate_idx_) - state_->ang_vel()(3);
    double glide_slope_rate_error = (sin(vars_.input(input_altitude_rate_idx_) / state_->vel().norm()) -
        sin(state_->vel()(3) / state_->vel().norm())) / dt;

    // Calculate the desired roll based on the ratio of heading_rate_error versus glide_slope_rate_error
    roll_rate_pid_.set_setpoint(atan2(heading_rate_error, glide_slope_rate_error));
    // Aileron control based on roll rate
    double u_roll_rate = roll_rate_pid_.step(dt, state_->quat().roll());
    // Desired pitch rate based on the magnitude of the heading and altitude errors
    pitch_rate_pid_.set_setpoint(sqrt(heading_rate_error * heading_rate_error +
        glide_slope_rate_error * glide_slope_rate_error));
    Eigen::Vector3d local_ang_vel(state_->ang_vel()(0), -state_->ang_vel()(1), -state_->ang_vel()(2));
    local_ang_vel = quat_body_.rotate_reverse(local_ang_vel);
    double u_pitch_rate = pitch_rate_pid_.step(dt, local_ang_vel(1));

    


    std::cout << "Elevator Command: " << u_pitch_rate << ", Aileron Command: " <<
        u_roll_rate << ", Rudder Command: " << 0.0 << ", Throttle Command: " << 0.0 << std::endl;

    // Output the controls
    vars_.output(elevator_idx_, u_pitch_rate);
    vars_.output(aileron_idx_, u_roll_rate);
    // TODO: This should be driving sideslip to zero for a coordinated turn
    vars_.output(rudder_idx_, 0.0);
    vars_.output(throttle_idx_, 0.0);  // Always 0 for a gliding vehicle

    return true;
}
}  // namespace controller
}  // namespace scrimmage
