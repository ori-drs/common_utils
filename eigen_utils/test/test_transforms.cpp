#include <eigen_utils/eigen_rigidbody.hpp>
#include <iostream>
#include <functional>
#include <chrono>  // for high_resolution_clock
#include <vector>
#include <gtest/gtest.h>

// implementations from icp_odometry / pronto_math
void altQuatToRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  double test = q.w() * q.y() - q.z() * q.x();
  double sqy = q.y() * q.y();

  if(test > 0.4999999){
    rpy << 0, 0.5*M_PI, 2 * atan2(q.y(), q.x());
    return;
  }

  if(test < -0.4999999){
    rpy << 0, -0.5*M_PI, -2 * atan2(q.y(), q.x());
    return;
  }

  rpy << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + sqy)),
         asin(2.0 * test),
         atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (sqy + q.z() * q.z()));
}

Eigen::Quaterniond altRPYtoQuat(const Eigen::Vector3d& rpy) {
    //Eigen::Quaterniond q;
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0

  if (rpy(0) == M_PI && rpy(1) == 0 && rpy(2) == 0){
    return Eigen::Quaterniond(0, 1, 0, 0);
  } else if(rpy(1) == M_PI && rpy(0) == 0 && rpy(2) == 0){
    return Eigen::Quaterniond(0, 0, 1, 0);
  } else if(rpy(2) == M_PI && rpy(0) == 0 && rpy(1) == 0){
    return  Eigen::Quaterniond(0, 0, 0, 1);
  }

  return Eigen::Quaterniond(cos(rpy(1)*0.5)*cos(rpy(1)*0.5)*cos(rpy(2)*0.5) + sin(rpy(1)*0.5)*sin(rpy(1)*0.5)*sin(rpy(2)*0.5),
                            sin(rpy(1)*0.5)*cos(rpy(1)*0.5)*cos(rpy(2)*0.5) - cos(rpy(1)*0.5)*sin(rpy(1)*0.5)*sin(rpy(2)*0.5),
                            cos(rpy(1)*0.5)*sin(rpy(1)*0.5)*cos(rpy(2)*0.5) + sin(rpy(1)*0.5)*cos(rpy(1)*0.5)*sin(rpy(2)*0.5),
                            cos(rpy(1)*0.5)*cos(rpy(1)*0.5)*sin(rpy(2)*0.5) - sin(rpy(1)*0.5)*sin(rpy(1)*0.5)*cos(rpy(2)*0.5));
}

void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2));
  pitch = asin(2.0*(q0*q2-q3*q1));
  yaw = atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
}

void icpQuatToRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  quat_to_euler(q, rpy(0),rpy(1),rpy(2));
}

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {

  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }

  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

Eigen::Quaterniond icpRPYtoQuat(const Eigen::Vector3d& rpy) {
  return euler_to_quat(rpy(0),rpy(1),rpy(2));
}

void eigenUtilsQuatToRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy){
  rpy = eigen_utils::getEulerAngles(q);
}

Eigen::Quaterniond eigenUtilsRPYtoQuat(const Eigen::Vector3d& rpy){
  return eigen_utils::setQuatEulerAngles(rpy);
}

void tfQuatToRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy){
  double roll, pitch, yaw;
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x()*q.z() - q.w()*q.y()) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  if (sarg <= -0.99999) {
    pitch = -0.5*M_PI;
    roll  = 0;
    yaw   = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    pitch = 0.5*M_PI;
    roll  = 0;
    yaw   = 2 * atan2(q.y(), q.x());
  } else {
    pitch = asin(sarg);
    roll  = atan2(2 * (q.y()*q.z() + q.w()*q.x()), sqw - sqx - sqy + sqz);
    yaw   = atan2(2 * (q.x()*q.y() + q.w()*q.z()), sqw + sqx - sqy - sqz);
  }
  rpy << roll, pitch, yaw;
}

Eigen::Quaterniond tfRPYtoQuat(const Eigen::Vector3d& rpy){
  Eigen::Quaterniond quat;
  double halfYaw =   rpy(2)   * 0.5;
  double halfPitch = rpy(1) * 0.5;
  double halfRoll =  rpy(0)  * 0.5;
  double cosYaw =    cos(halfYaw);
  double sinYaw =    sin(halfYaw);
  double cosPitch =  cos(halfPitch);
  double sinPitch =  sin(halfPitch);
  double cosRoll =   cos(halfRoll);
  double sinRoll =   sin(halfRoll);
  quat.x() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; //x
  quat.y() = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; //y
  quat.z() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; //z
  quat.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; //formerly yzx
  return quat;
}

double time_q2rpy_implementation(std::function<void(const Eigen::Quaterniond&,Eigen::Vector3d&)> q2e,
                             std::string name,
                             size_t iterations = 1000){
  std::cout << "Timing " << name << " Quaternion to RPY..." << std::endl ;
  auto start  = std::chrono::high_resolution_clock::now();
  for(size_t i = 0; i < iterations; i++){
    Eigen::Vector3d rpy_out;
    auto quat = Eigen::Quaterniond::UnitRandom();
    q2e(quat, rpy_out);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time for " << name << " Quat to RPY: " << elapsed.count() << std::endl;
  std::cerr << "---------------" << std::endl;
  return elapsed.count();
}

double time_rpy2q_implementation(std::function<Eigen::Quaterniond(const Eigen::Vector3d&)> e2q,
                             std::string name,
                             double ang_increment = 0.1){
  std::cout << "Timing " << name << " RPY to Quat..." << std::endl ;
  double ang_start = -M_PI;
  double ang_end = M_PI;

  //ang_increment = M_PI / 360.0;

  auto start  = std::chrono::high_resolution_clock::now();
  for(double roll = ang_start; roll <= ang_end; roll = roll + ang_increment){
    for(double pitch = ang_start; pitch <= ang_end; pitch = pitch + ang_increment){
      for(double yaw = ang_start; yaw <= ang_end; yaw = yaw + ang_increment){
        Eigen::Vector3d rpy(roll, pitch, yaw);
        auto quat = e2q(rpy);
      }
    }
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time for " << name << " RPY to Quat : " << elapsed.count() << std::endl;
  std::cerr << "---------------" << std::endl;
  return elapsed.count();
}



void test_implementation(std::function<Eigen::Quaterniond(const Eigen::Vector3d&)> e2q,
                         std::function<void(const Eigen::Quaterniond&,Eigen::Vector3d&)> q2e,
                         std::string name,
                         double ang_increment = 0.1,
                         bool debug_prints = true){
  std::cout << "Evaluating " << name <<  "..." << std::endl ;
  double ang_start = -M_PI;
  double ang_end = M_PI;

  for(double roll = ang_start; roll <= ang_end; roll = roll + ang_increment){
    for(double pitch = ang_start; pitch <= ang_end; pitch = pitch + ang_increment){
      for(double yaw = ang_start; yaw <= ang_end; yaw = yaw + ang_increment){
        double roll_out, pitch_out, yaw_out;
        Eigen::Vector3d rpy_out;
        Eigen::Vector3d rpy(roll, pitch, yaw);
        Eigen::Quaterniond quat = e2q(rpy);

        q2e(quat, rpy_out);
        roll_out =  rpy_out(0);
        pitch_out = rpy_out(1);
        yaw_out =   rpy_out(2);

        Eigen::Quaterniond quat_out = e2q(rpy_out);
        Eigen::Quaterniond quat_resid = quat.inverse() * quat_out;
        quat_resid.normalize();

        Eigen::Vector3d diff = eigen_utils::subtractQuats(quat, quat_out);

        double tol = 1e-6;
        EXPECT_NEAR(diff.norm(), 0, tol);
        if(debug_prints){
          if(diff.norm() > tol){
            std::cerr << "---------------" << std::endl;
            std::cerr << "[ " << name << " ] WARNING: not the same angle" << std::endl;
            std::cout << "[ " << name << " ] WARNING: quat     " << quat.w() << " "<< quat.x() << " "<< quat.y() << " "<< quat.z() << std::endl;
            std::cout << "[ " << name << " ] WARNING: quat_out "<< quat_out.w() << " "<< quat_out.x() << " "<< quat_out.y() << " "<< quat_out.z() << std::endl;
            std::cout << "[ " << name << " ] WARNING: resid    "  << quat_resid.w() << " "<< quat_resid.x() << " "<< quat_resid.y() << " "<< quat_resid.z() << std::endl;


            std::cerr << "[ " << name << " ] WARNING: diff  " << diff.transpose() << std::endl;
            std::cerr << "[ " << name << " ] WARNING: roll  " << roll * 180.0 / M_PI<< " != " << roll_out * 180.0 / M_PI << std::endl;
            std::cerr << "[ " << name << " ] WARNING: pitch " << pitch * 180.0 / M_PI<< " != " << pitch_out * 180.0 / M_PI<< std::endl;
            std::cerr << "[ " << name << " ] WARNING: yaw   "  << yaw * 180.0 / M_PI<< " != " << yaw_out * 180.0 / M_PI<< std::endl;
          }

          if(std::abs(roll - roll_out) > tol){
            std::cerr << "[ " << name << " ] WARNING: roll " << roll * 180.0 / M_PI<< " != " << roll_out * 180.0 / M_PI << std::endl;
          }
          if(std::abs(pitch - pitch_out) > tol){
            std::cerr << "[ " << name << " ] WARNING: pitch " << pitch * 180.0 / M_PI<< " != " << pitch_out * 180.0 / M_PI<< std::endl;
          }
          if(std::abs(yaw - yaw_out) > tol ){
            std::cerr << "[ " << name << " ] WARNING: yaw " << yaw * 180.0 / M_PI<< " != " << yaw_out * 180.0 / M_PI<< std::endl;
          }
        }
      }
    }
  }
}

void test_singularities(std::function<Eigen::Quaterniond(const Eigen::Vector3d&)> e2q,
                     std::function<void(const Eigen::Quaterniond&,Eigen::Vector3d&)> q2e,
                     std::string name)
{

  std::cout << "Testing bad values for " << name << std::endl;

  std::vector<double> singular_values = {M_PI, -M_PI, 0, M_PI_2, -M_PI_2, M_PI_4, -M_PI_4, 2*M_PI, -2*M_PI};

  for(size_t i = 0; i < singular_values.size(); i++){
    for(size_t j = 0; j < singular_values.size(); j++){
      for(size_t k = 0; k < singular_values.size(); k++){

        Eigen::Vector3d singular_vector(singular_values[i],singular_values[j],singular_values[k]);

        double roll, pitch, yaw;
        Eigen::Vector3d rpy_out;
        Eigen::Quaterniond quat = e2q(singular_vector);
        q2e(quat, rpy_out);
        roll = rpy_out(0);
        pitch = rpy_out(1);
        yaw = rpy_out(2);

        EXPECT_TRUE(rpy_out.allFinite());

        if(!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)){
          std::cerr << "ERROR " << name << " using " << singular_values[i] << " " << singular_values[j] << " " <<  singular_values[k] << " yields NaN or Inf" << std::endl;
        }
      }
    }
  }
}

TEST(QuaternionRPYConversions, singularities){
  test_singularities(eigenUtilsRPYtoQuat, eigenUtilsQuatToRPY, "eigen_utils");
  // alternative functions to test
  // test_singularities(altRPYtoQuat, altQuatToRPY,               "alternative");
  // test_singularities(icpRPYtoQuat, icpQuatToRPY,               "icp_odom   ");
  // test_singularities(tfRPYtoQuat, tfQuatToRPY,                 "tf         ");
}

TEST(QuaternionRPYConversions, selfConsistency){
  double ang_increment = M_PI / 36.0;
  bool print = false;
  test_implementation(eigenUtilsRPYtoQuat, eigenUtilsQuatToRPY, "eigen_utils",  ang_increment, print);
  // alternative functions to test
  // test_implementation(altRPYtoQuat, altQuatToRPY,               "alternative",  ang_increment, print);
  // test_implementation(icpRPYtoQuat, icpQuatToRPY,               "icp_odom   ",  ang_increment, print);
  // test_implementation(tfRPYtoQuat, tfQuatToRPY,                 "tf         ",  ang_increment, print);

}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  // Time various alternatives
  double ang_increment = M_PI / 36.0;
  time_rpy2q_implementation(altRPYtoQuat,          "alternative", ang_increment);
  time_rpy2q_implementation(icpRPYtoQuat,          "icp_odom   ", ang_increment);
  time_rpy2q_implementation(tfRPYtoQuat,           "tf         ", ang_increment);
  time_rpy2q_implementation(eigenUtilsRPYtoQuat,   "eigen_utils", ang_increment);

  size_t iters = 1e3;
  time_q2rpy_implementation(altQuatToRPY,        "alternative", iters);
  time_q2rpy_implementation(icpQuatToRPY,        "icp_odom   ", iters);
  time_q2rpy_implementation(tfQuatToRPY,         "tf         ", iters);
  time_q2rpy_implementation(eigenUtilsQuatToRPY, "eigen_utils", iters);

  return RUN_ALL_TESTS();
}
