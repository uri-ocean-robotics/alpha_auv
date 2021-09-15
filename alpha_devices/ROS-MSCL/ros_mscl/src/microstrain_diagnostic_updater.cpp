#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "microstrain_diagnostic_updater.h"
#include "microstrain_3dm.h"
#include <string>



namespace ros_mscl
{

RosDiagnosticUpdater::RosDiagnosticUpdater()
{
  setHardwareID("unknown");
  add("general", this, &RosDiagnosticUpdater::generalDiagnostics);
  add("packet", this, &RosDiagnosticUpdater::packetDiagnostics);
  add("port", this, &RosDiagnosticUpdater::portDiagnostics);
  add("imu", this, &RosDiagnosticUpdater::imuDiagnostics);



  status_sub_ = nh_.subscribe("device/status", 5, &RosDiagnosticUpdater::statusCallback, this);
}


void RosDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Device Model", last_status_.device_model);
  stat.add("Status Selector", last_status_.status_selector);
  stat.add("Status Flags", last_status_.status_flags);
  stat.add("System Timer ms", last_status_.system_timer_ms);
  stat.add("IMU Stream Enabled", last_status_.imu_stream_enabled);
  stat.add("Filter Stream Enabled", last_status_.filter_stream_enabled);

  if (last_status_.status_flags > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Status flags raised");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status ok");
  }
  
}

void RosDiagnosticUpdater::packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("IMU Dropped Packets", last_status_.imu_dropped_packets);
  stat.add("Filter Dropped Packets", last_status_.filter_dropped_packets);


  if (last_status_.imu_dropped_packets > 0 || last_status_.filter_dropped_packets > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Packets dropped");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No dropped packets");
  }
}

void RosDiagnosticUpdater::portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("COM1 Port Bytes Written", last_status_.com1_port_bytes_written);
  stat.add("COM1 Port Bytes Read", last_status_.com1_port_bytes_read);
  stat.add("COM1 Port Write Overruns", last_status_.com1_port_write_overruns);
  stat.add("COM1 Port Read Overruns", last_status_.com1_port_read_overruns);

  if (last_status_.com1_port_write_overruns > 0 || last_status_.com1_port_read_overruns > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Port overruns");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No port overruns");
  }
}

void RosDiagnosticUpdater::imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("IMU Parser Errors", last_status_.imu_parser_errors);
  stat.add("IMU Message Count", last_status_.imu_message_count);
  stat.add("IMU Last Message ms", last_status_.imu_last_message_ms);

  if (last_status_.imu_parser_errors > 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "IMU Parser Errors");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No IMU parser errors");
  }
}


void RosDiagnosticUpdater::statusCallback(const mscl_msgs::Status::ConstPtr& status)
{
  last_status_ = *status;
  update();
}


}
