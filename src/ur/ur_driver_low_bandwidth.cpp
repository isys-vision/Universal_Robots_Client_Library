// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Many parts from this (Most of the URScript program) comes from the ur_modern_driver
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/ur/ur_driver_low_bandwidth.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/primary/primary_parser.h"
#include <memory>
#include <sstream>

#include <ur_client_library/ur/calibration_checker.h>

namespace urcl
{

static const std::array<double, 6> EMPTY_VALUES = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

static const std::string BEGIN_REPLACE("{{BEGIN_REPLACE}}");
static const std::string SERVOJ_TIME("{{SERVOJ_TIME}}");
static const std::string SERVOJ_TIME_WAITING("{{SERVOJ_TIME_WAITING}}");
static const std::string SERVOJ_GAIN("{{SERVOJ_GAIN}}");
static const std::string SERVOJ_LOOKAHEAD_TIME("{{SERVOJ_LOOKAHEAD_TIME}}");
static const std::string REVERSE_IP("{{REVERSE_IP}}");
static const std::string REVERSE_PORT("{{REVERSE_PORT}}");
static const std::string MAX_JOINT_DIFFERENCE("{{MAX_JOINT_DIFFERENCE}}");

urcl::UrDriverLowBandwidth::UrDriverLowBandwidth(const std::string& robot_ip, const std::string& script_file,
                              const std::string& output_recipe_file, const std::string& input_recipe_file,
                              std::function<void(bool)> handle_program_state, bool headless_mode,
                              std::unique_ptr<ToolCommSetup> tool_comm_setup, const std::string& calibration_checksum,
                              const uint32_t reverse_port, const uint32_t script_sender_port, double servoj_time_waiting, int servoj_gain,
                              double servoj_lookahead_time, bool non_blocking_read,
                              double max_joint_difference, double max_velocity)
    : servoj_time_(0.002)
    , servoj_time_waiting_(servoj_time_waiting)
    , servoj_gain_(servoj_gain)
    , servoj_lookahead_time_(servoj_lookahead_time)
    , max_joint_difference_(max_joint_difference)
    , max_velocity_ (max_velocity)
    , reverse_interface_active_(false)
    , reverse_port_(reverse_port)
    , handle_program_state_(handle_program_state)
    , robot_ip_(robot_ip)
{
  LOG_DEBUG("Initializing urdriver");
  LOG_DEBUG("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip_, notifier_, output_recipe_file, input_recipe_file));

  primary_stream_.reset(
      new comm::URStream<primary_interface::PrimaryPackage>(robot_ip_, urcl::primary_interface::UR_PRIMARY_PORT));
  secondary_stream_.reset(
      new comm::URStream<primary_interface::PrimaryPackage>(robot_ip_, urcl::primary_interface::UR_SECONDARY_PORT));
  secondary_stream_->connect();
  LOG_INFO("Checking if calibration data matches connected robot.");
  checkCalibration(calibration_checksum);

  non_blocking_read_ = non_blocking_read;
  get_packet_timeout_ = non_blocking_read_ ? 0 : 100;

  if (!rtde_client_->init())
  {
    throw UrException("Initialization of RTDE client went wrong.");
  }

  std::string local_ip = rtde_client_->getIP();
  robot_version_ = rtde_client_->getVersion();

  // older UR controllers use lower update rates
  if (robot_version_.major < 5) servoj_time_ = 0.008;
  else servoj_time_ = 0.002;

  LOG_INFO("UR control version: %i.%i.%i-%i", robot_version_.major, robot_version_.minor, robot_version_.bugfix, robot_version_.build);
  LOG_INFO("Used parameters (UR script):");
  LOG_INFO("  servoj_time %f, servoj_time_waiting %f, "
           "servoj_gain: %f, servoj_lookahead_time: %f, max_joint_difference: %f, max_velocity: %f",
           servoj_time_, servoj_time_waiting_, servoj_gain_, servoj_lookahead_time_, max_joint_difference_, max_velocity_);

  std::string prog = readScriptFile(script_file);
  while (prog.find(SERVOJ_TIME_WAITING) != std::string::npos)
  {
    prog.replace(prog.find(SERVOJ_TIME_WAITING), SERVOJ_TIME_WAITING.length(), std::to_string(servoj_time_waiting_));
  }

  while (prog.find(SERVOJ_TIME) != std::string::npos)
  {
    prog.replace(prog.find(SERVOJ_TIME), SERVOJ_TIME.length(), std::to_string(servoj_time_));
  }

  while (prog.find(SERVOJ_GAIN) != std::string::npos)
  {
    prog.replace(prog.find(SERVOJ_GAIN), SERVOJ_GAIN.length(), std::to_string(servoj_gain_));
  }

  while (prog.find(SERVOJ_LOOKAHEAD_TIME) != std::string::npos)
  {
    prog.replace(prog.find(SERVOJ_LOOKAHEAD_TIME), SERVOJ_LOOKAHEAD_TIME.length(), std::to_string(servoj_lookahead_time_));
  }

  while (prog.find(REVERSE_IP) != std::string::npos)
  {
    prog.replace(prog.find(REVERSE_IP), REVERSE_IP.length(), local_ip);
  }

  while (prog.find(REVERSE_PORT) != std::string::npos)
  {
    prog.replace(prog.find(REVERSE_PORT), REVERSE_PORT.length(), std::to_string(reverse_port));
  }

  while (prog.find(MAX_JOINT_DIFFERENCE) != std::string::npos)
  {
    prog.replace(prog.find(MAX_JOINT_DIFFERENCE), MAX_JOINT_DIFFERENCE.length(), std::to_string(max_joint_difference_));
  }

  std::stringstream begin_replace;
  if (tool_comm_setup != nullptr)
  {
    if (robot_version_.major < 5)
    {
      throw ToolCommNotAvailable("Tool communication setup requested, but this robot version does not support using "
                                 "the tool communication interface. Please check your configuration.",
                                 5, robot_version_.major);
    }
    begin_replace << "set_tool_voltage("
                  << static_cast<std::underlying_type<ToolVoltage>::type>(tool_comm_setup->getToolVoltage()) << ")\n";
    begin_replace << "set_tool_communication("
                  << "True"
                  << ", " << tool_comm_setup->getBaudRate() << ", "
                  << static_cast<std::underlying_type<Parity>::type>(tool_comm_setup->getParity()) << ", "
                  << tool_comm_setup->getStopBits() << ", " << tool_comm_setup->getRxIdleChars() << ", "
                  << tool_comm_setup->getTxIdleChars() << ")";
  }
  prog.replace(prog.find(BEGIN_REPLACE), BEGIN_REPLACE.length(), begin_replace.str());

  in_headless_mode_ = headless_mode;
  if (in_headless_mode_)
  {
    full_robot_program_ = "def externalControl():\n";
    std::istringstream prog_stream(prog);
    std::string line;
    while (std::getline(prog_stream, line))
    {
      full_robot_program_ += "\t" + line + "\n";
    }
    full_robot_program_ += "end\n";
    sendRobotProgram();
  }
  else
  {
    script_sender_.reset(new comm::ScriptSender(script_sender_port, prog));
    script_sender_->start();
    LOG_DEBUG("Created script sender");
  }

  reverse_port_ = reverse_port;

  // watchdog (and keepalive) not required for low bandwidth driver at this point
//  watchdog_thread_ = std::thread(&UrDriverLowBandwidth::startWatchdog, this);

  LOG_DEBUG("Initialization done");
}

std::unique_ptr<rtde_interface::DataPackage> urcl::UrDriverLowBandwidth::getDataPackage()
{
  // This can take one of two values, 0ms or 100ms. The large timeout is for when the robot is commanding the control
  // loop's timing (read is blocking). The zero timeout is for when the robot is sharing a control loop with
  // something else (combined_robot_hw)
  std::chrono::milliseconds timeout(get_packet_timeout_);

  return rtde_client_->getDataPackage(timeout);
}

bool UrDriverLowBandwidth::writeJointCommand(const vector6d_t& values, const comm::ControlMode control_mode)
{
  if (reverse_interface_active_)
  {
    return reverse_interface_->write(&values, control_mode);
  }
  return false;
}

bool UrDriverLowBandwidth::writeKeepalive()
{
  if (reverse_interface_active_)
  {
    vector6d_t* fake = nullptr;
    return reverse_interface_->write(fake, comm::ControlMode::MODE_IDLE);
  }
  return false;
}

void UrDriverLowBandwidth::startRTDECommunication()
{
  rtde_client_->start();
}

bool UrDriverLowBandwidth::stopControl()
{
  if (reverse_interface_active_)
  {
    vector6d_t* fake = nullptr;
    return reverse_interface_->write(fake, comm::ControlMode::MODE_STOPPED);
  }
  return false;
}

void UrDriverLowBandwidth::startWatchdog()
{
  handle_program_state_(false);
  reverse_interface_.reset(new comm::ReverseInterface(reverse_port_));
  reverse_interface_active_ = true;
  LOG_DEBUG("Created reverse interface");

  while (true)
  {
    LOG_INFO("Robot ready to receive control commands.");
    handle_program_state_(true);
    while (reverse_interface_active_ == true)
    {
      std::string keepalive = readKeepalive();

      if (keepalive == std::string(""))
      {
        reverse_interface_active_ = false;
      }
    }

    LOG_INFO("Connection to robot dropped, waiting for new connection.");
    handle_program_state_(false);
    // We explicitly call the destructor here, as unique_ptr.reset() creates a new object before
    // replacing the pointer and destroying the old object. This will result in a resource conflict
    // when trying to bind the socket.
    // TODO: It would probably make sense to keep the same instance alive for the complete runtime
    // instead of killing it all the time.
    reverse_interface_->~ReverseInterface();
    reverse_interface_.reset(new comm::ReverseInterface(reverse_port_));
    reverse_interface_active_ = true;
  }
}

std::string UrDriverLowBandwidth::readScriptFile(const std::string& filename)
{
  std::ifstream ifs(filename);
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return content;
}
std::string UrDriverLowBandwidth::readKeepalive()
{
  if (reverse_interface_active_)
  {
    return reverse_interface_->readKeepalive();
  }
  else
  {
    return std::string("");
  }
}

void UrDriverLowBandwidth::checkCalibration(const std::string& checksum)
{
  if (primary_stream_ == nullptr)
  {
    throw std::runtime_error("checkCalibration() called without a primary interface connection being established.");
  }
  primary_interface::PrimaryParser parser;
  comm::URProducer<primary_interface::PrimaryPackage> prod(*primary_stream_, parser);
  prod.setupProducer();

  CalibrationChecker consumer(checksum);

  comm::INotifier notifier;

  comm::Pipeline<primary_interface::PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
  pipeline.run();

  while (!consumer.isChecked())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  LOG_DEBUG("Got calibration information from robot.");
}

rtde_interface::RTDEWriter& UrDriverLowBandwidth::getRTDEWriter()
{
  return rtde_client_->getWriter();
}

bool UrDriverLowBandwidth::sendScript(const std::string& program)
{
  if (secondary_stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = program + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (secondary_stream_->write(data, len, written))
  {
    LOG_DEBUG("Sent program to robot:\n%s", program_with_newline.c_str());
    return true;
  }
  LOG_ERROR("Could not send program to robot");
  return false;
}

bool UrDriverLowBandwidth::sendRobotProgram()
{
  if (in_headless_mode_)
  {
    return sendScript(full_robot_program_);
  }
  else
  {
    LOG_ERROR("Tried to send robot program directly while not in headless mode");
    return false;
  }
}
}  // namespace urcl
