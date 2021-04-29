
// ros
#include <ros/ros.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/static_object.hpp"

using namespace flightlib;


Vector<3> getPosition(Scalar t) {
  return Vector<3>(
    std::cos(t),
    std::sin(t),
    2 + 0.5*std::cos(2*t)
  );
}


int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "objects_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::WAREHOUSE};
  bool unity_ready{false};

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  QuadState quad_state;
  quad_state.setZero();
  quad_state.x[QS::POSY] = -4;
  quad_state.x[QS::POSZ] = 2.5;
  quad_ptr->reset(quad_state);
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);
  unity_bridge_ptr->addQuadrotor(quad_ptr);

  // Initialize a cube that will be streched and moved around
  std::shared_ptr<StaticObject> cube = std::make_shared<StaticObject>("cube", "Transparen_Cube");
  cube->setPosition(Vector<3>(0, 0, 1.0));
  cube->setQuaternion(Quaternion(1, 0, 0, 0));
  cube->setSize(Vector<3>(3, 1, 1));
  unity_bridge_ptr->addStaticObject(cube);

  // These will be used later
  std::vector<std::shared_ptr<StaticObject>> drones;
  std::vector<Scalar> tspawn;

  // Start the "simulation"
  ros::Time t0 = ros::Time::now();

  // connect unity
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;
  const FrameID frame_spacing = 100;
  const int max_drones = 10;

  while (ros::ok() && unity_ready) {
    Scalar t = static_cast<Scalar>( (ros::Time::now() - t0).toSec() );

    cube->setSize(Vector<3>(5*(0.6 + 0.4*std::sin(t)), 0.5, 0.5));
    Scalar rho = t/(2*std::sqrt(2));
    cube->setQuaternion(Quaternion(std::cos(rho),0,0,std::sin(rho)));

    if(drones.size() == max_drones) {
      bool removed = unity_bridge_ptr->removeStaticObject(drones.at(0)->getID());
      if(!removed) {
        ROS_WARN_STREAM("Failed to remove '" << drones.at(0)->getID() << "' from the scene");
      }
      else {
        drones.erase(drones.begin());
        tspawn.erase(tspawn.begin());
      }
    }

    if(frame_id > 0 && frame_id % frame_spacing == 0 && drones.size() < max_drones) {
      tspawn.push_back(t);
      std::string drone_name = "drone" + std::to_string(frame_id/frame_spacing);
      std::shared_ptr<StaticObject> drone = std::make_shared<StaticObject>(drone_name, "Drone_red");
      drone->setPosition(getPosition(0));
      drone->setQuaternion(Quaternion(1,0,0,0));
      unity_bridge_ptr->addStaticObject(drone);
      drones.push_back(drone);
    }

    for(unsigned int i=0; i<drones.size(); i++) {
      drones[i]->setPosition(getPosition(t-tspawn.at(i)));
    }

    Vector<3> quad_size(
      1+0.5*std::sin(5*t),
      1-0.5*std::sin(5*t),
      1
    );
    quad_ptr->setSize(quad_size);

    unity_bridge_ptr->getRender(frame_id);
    unity_bridge_ptr->handleOutput();

    //
    frame_id += 1;
  }

  return 0;
}
