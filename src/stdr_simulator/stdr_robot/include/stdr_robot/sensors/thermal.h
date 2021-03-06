/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

   Authors :
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com
******************************************************************************/

#ifndef THERMAL_SENSOR_H
#define THERMAL_SENSOR_H

#include <stdr_msgs/ThermalSensorMeasurementMsg.h>
#include <stdr_msgs/ThermalSensorMsg.h>
#include <stdr_msgs/ThermalSourceVector.h>
#include <stdr_robot/sensors/helper.h>
#include <stdr_robot/sensors/sensor_base.h>

/**
@namespace stdr_robot
@brief The main namespace for STDR Robot
**/
namespace stdr_robot {

/**
@class Sonar
@brief A class that provides thermal sensor implementation. \
Inherits publicly Sensor
**/
class ThermalSensor : public Sensor {
 public:
  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::ThermalSensorMsg&] The thermal sensor \
  description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/
  ThermalSensor(const nav_msgs::OccupancyGrid& map,
                const stdr_msgs::ThermalSensorMsg& msg, const std::string& name,
                ros::NodeHandle& n);

  /**
  @brief Updates the sensor measurements
  @return void
  **/
  virtual void updateSensorCallback();

  /**
  @brief Default destructor
  @return void
  **/
  ~ThermalSensor(void);

  /**
  @brief Receives the existent sound sources
  @param msg [const stdr_msgs::ThermalSourceVector&] The thermal sources message
  @return void
  **/
  void receiveThermalSources(const stdr_msgs::ThermalSourceVector& msg);

 private:
  //!< thermal sensor description
  stdr_msgs::ThermalSensorMsg _description;

  //!< ROS subscriber for sound sources
  ros::Subscriber thermal_sources_subscriber_;

  //!< The currently existent sources
  stdr_msgs::ThermalSourceVector thermal_sources_;
};

}  // namespace stdr_robot

#endif
