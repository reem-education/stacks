#include "gazebo_ros_range.h"
#include <algorithm>
#include <string>
#include <assert.h>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/HingeJoint.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sdf/interface/SDF.hh"
#include "gazebo/sdf/interface/Param.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorTypes.hh"

#include "tf/tf.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRange)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosRange::GazeboRosRange()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosRange::~GazeboRosRange()
{
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRange::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;
  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->last_update_time_ = common::Time(0);

  this->parent_ray_sensor_ =
    boost::shared_dynamic_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosRange controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->GetValueString("robotNamespace") + "/";

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->GetValueString("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->GetValueString("topicName");

  if (!this->sdf->HasElement("radiation"))
  {
      ROS_WARN("Range plugin missing <radiation>, defaults to ultrasound");
      this->radiation_ = "ultrasound";

  }
  else
      this->radiation_ = _sdf->GetElement("radiation")->GetValueString();

  if (!this->sdf->HasElement("fov"))
  {
      ROS_WARN("Range plugin missing <fov>, defaults to 0.05");
      this->fov_ = 0.05;
  }
  else
      this->fov_ = _sdf->GetElement("fov")->GetValueDouble();
  if (!this->sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = this->sdf->GetValueDouble("gaussianNoise");



  if (!this->sdf->GetElement("updateRate"))
  {
    ROS_INFO("Laser plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->GetValueDouble("updateRate");

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->range_connect_count_ = 0;

  this->range_msg_.header.frame_id = this->frame_name_;
  if (this->radiation_==std::string("ultrasound"))
     this->range_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  else
      this->range_msg_.radiation_type = sensor_msgs::Range::INFRARED;

  this->range_msg_.field_of_view = fov_;
  this->range_msg_.max_range = this->parent_ray_sensor_->GetRangeMax();
  this->range_msg_.min_range = this->parent_ray_sensor_->GetRangeMin();

  // Init ROS
  if (ros::isInitialized())
  {
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
      boost::bind(&GazeboRosRange::LoadThread, this));
  }
  else
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRange::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::Range>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosRange::RangeConnect, this),
      boost::bind(&GazeboRosRange::RangeDisconnect, this),
      ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosRange::RangeQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosRange::RangeConnect()
{
  this->range_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosRange::RangeDisconnect()
{
  this->range_connect_count_--;

  if (this->range_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosRange::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time =
        this->parent_sensor_->GetLastUpdateTime();
      this->PutRangeData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_laser topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosRange::PutRangeData(common::Time &_updateTime)
{
//  int i, ja, jb;
//  double ra, rb, r, b;
//  double intensity;

  this->parent_ray_sensor_->SetActive(false);

//  math::Angle maxAngle = this->parent_ray_sensor_->GetAngleMax();
//  math::Angle minAngle = this->parent_ray_sensor_->GetAngleMin();

//  double maxRange = this->parent_ray_sensor_->GetRangeMax();
//  double minRange = this->parent_ray_sensor_->GetRangeMin();
//  int rayCount = this->parent_ray_sensor_->GetRayCount();
//  int rangeCount = this->parent_ray_sensor_->GetRangeCount();

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  {
    boost::mutex::scoped_lock lock(this->lock_);
    // Add Frame Name
    this->range_msg_.header.frame_id = this->frame_name_;
    this->range_msg_.header.stamp.sec = _updateTime.sec;
    this->range_msg_.header.stamp.nsec = _updateTime.nsec;


//    // for computing yaw
//    double tmp_res_angle = (maxAngle.Radian() -
//      minAngle.Radian())/(static_cast<double>(rangeCount -1));
//    this->laser_msg_.angle_min = minAngle.Radian();
//    this->laser_msg_.angle_max = maxAngle.Radian();
//    this->laser_msg_.angle_increment = tmp_res_angle;
//    this->laser_msg_.time_increment  = 0;  // instantaneous simulator scan
//    this->laser_msg_.scan_time       = 0;  // FIXME: what's this?
//    this->laser_msg_.range_min = minRange;
//    this->laser_msg_.range_max = maxRange;
//    this->laser_msg_.ranges.clear();
//    this->laser_msg_.intensities.clear();

//    // Interpolate the range readings from the rays
//    for (i = 0; i < rangeCount; ++i)
//    {
//      b = static_cast<double>(i * (rayCount - 1) / (rangeCount - 1));
//      ja = static_cast<int>(floor(b));
//      jb = std::min(ja + 1, rayCount - 1);
//      b = b - floor(b);

//      assert(ja >= 0 && ja < rayCount);
//      assert(jb >= 0 && jb < rayCount);

//      ra = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(ja),
//        maxRange-minRange);  // length of ray
//      rb = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(jb),
//        maxRange-minRange);  // length of ray

//      // Range is linear interpolation if values are close,
//      // and min if they are very different
//      // if (fabs(ra - rb) < 0.10)
//      r = (1 - b) * ra + b * rb;
//      // else r = std::min(ra, rb);

//      // Intensity is averaged
//      intensity = 0.5*(this->parent_ray_sensor_->GetLaserShape()->GetRetro(ja)
//                     + this->parent_ray_sensor_->GetLaserShape()->GetRetro(jb));

//      /***************************************************************/
//      /*                                                             */
//      /*  point scan from laser                                      */
//      /*                                                             */
//      /***************************************************************/
//      this->laser_msg_.ranges.push_back(std::min(r + minRange +
//        this->GaussianKernel(0, this->gaussian_noise_), maxRange));
//      this->laser_msg_.intensities.push_back(
//        std::max(this->hokuyo_min_intensity_,
//                 intensity + this->GaussianKernel(0, this->gaussian_noise_)));
//    }

    // find ray with minimal range
    range_msg_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

    int num_ranges = parent_ray_sensor_->GetLaserShape()->GetSampleCount() * parent_ray_sensor_->GetLaserShape()->GetVerticalSampleCount();

    for(int i = 0; i < num_ranges; ++i)
    {
        double ray = parent_ray_sensor_->GetLaserShape()->GetRange(i);
        if (ray < range_msg_.range)
            range_msg_.range = ray;
    }

    // add Gaussian noise and limit to min/max range
    if (range_msg_.range < range_msg_.max_range)
        range_msg_.range = std::min(range_msg_.range + this->GaussianKernel(0,gaussian_noise_), parent_ray_sensor_->GetRangeMax());


    this->parent_ray_sensor_->SetActive(true);

    // send data out via ros message
    if (this->range_connect_count_ > 0 && this->topic_name_ != "")
        this->pub_.publish(this->range_msg_);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosRange::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put range data to the interface
void GazeboRosRange::RangeQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->range_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
