#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <random>
#include <cmath>

namespace gazebo {

class GpsPlugin : public ModelPlugin {
public:
  GpsPlugin() : dist_(0.0, 0.0) {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      ROS_ERROR("GpsPlugin: ROS not initialized");
      return;
    }

    model_ = model;
    nh_ = ros::NodeHandle(
      sdf->HasElement("rosNamespace")
        ? sdf->Get<std::string>("rosNamespace")
        : ""
    );

    // --- Read parameters ---
    updateRate_  = sdf->Get<double>("updateRate", 5.0).first;
    refLat_      = sdf->Get<double>("referenceLatitude", 0.0).first;
    refLon_      = sdf->Get<double>("referenceLongitude", 0.0).first;
    refAlt_      = sdf->Get<double>("referenceAltitude", 0.0).first;
    noiseStdDev_ = sdf->Get<double>("noiseStdDev", 0.0).first;
    frameId_     = sdf->Get<std::string>("frameId", "gps_link").first;
    topicName_   = sdf->Get<std::string>("topicName", "/gps").first;

    // --- Log them to console ---
    ROS_INFO("GpsPlugin parameters:");
    ROS_INFO("  updateRate:          %.2f Hz",        updateRate_);
    ROS_INFO("  referenceLatitude:   %.8f°",           refLat_);
    ROS_INFO("  referenceLongitude:  %.8f°",           refLon_);
    ROS_INFO("  referenceAltitude:   %.3f m",          refAlt_);
    ROS_INFO("  noiseStdDev:         %.3f m (ENU)",     noiseStdDev_);
    ROS_INFO("  frameId:             %s",              frameId_.c_str());
    ROS_INFO("  topicName:           %s",              topicName_.c_str());

    // prepare timing
    updatePeriod_   = 1.0 / updateRate_;
    lastUpdateTime_ = model_->GetWorld()->SimTime();

    // noise distribution in meters (ENU frame)
    dist_ = std::normal_distribution<double>(0.0, noiseStdDev_);
    generator_.seed(std::random_device{}());

    gpsPub_ = nh_.advertise<sensor_msgs::NavSatFix>(topicName_, 1);

    // hook into simulation loop
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GpsPlugin::OnUpdate, this)
    );

    ROS_INFO("GpsPlugin loaded: publishing %s at %.1f Hz",
             topicName_.c_str(), updateRate_);
  }

private:
  void OnUpdate() {
    common::Time now = model_->GetWorld()->SimTime();
    if ((now - lastUpdateTime_).Double() < updatePeriod_) return;
    lastUpdateTime_ = now;

    // get true pose
    auto pose = model_->WorldPose().Pos();
    double x = pose.X(), y = pose.Y(), z = pose.Z();

    // add Gaussian noise in meters (ENU)
    double x_n = x + dist_(generator_);
    double y_n = y + dist_(generator_);
    double z_n = z + dist_(generator_);

    // ENU → WGS84
    const double R = 6378137.0;  // Earth radius [m]
    double dLat = y_n / R;
    double dLon = x_n / (R * std::cos(refLat_ * M_PI/180.0));
    double lat  = refLat_ + dLat * 180.0/M_PI;
    double lon  = refLon_ + dLon * 180.0/M_PI;
    double alt  = refAlt_ + z_n;

    // fill NavSatFix
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time::now();
    fix.header.frame_id = frameId_;
    fix.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    fix.latitude  = lat;
    fix.longitude = lon;
    fix.altitude  = alt;

    double cov = noiseStdDev_ * noiseStdDev_;
    fix.position_covariance = {
      cov, 0,   0,
      0,   cov, 0,
      0,   0,   cov
    };
    fix.position_covariance_type =
      noiseStdDev_ > 0.0
        ? sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED
        : sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    gpsPub_.publish(fix);
  }

  // members
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;
  ros::NodeHandle nh_;
  ros::Publisher gpsPub_;

  double updateRate_, updatePeriod_;
  common::Time lastUpdateTime_;
  double refLat_, refLon_, refAlt_, noiseStdDev_;
  std::string frameId_, topicName_;

  std::default_random_engine         generator_;
  std::normal_distribution<double>   dist_;
};

GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)

}  // namespace gazebo
