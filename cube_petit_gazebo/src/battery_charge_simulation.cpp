#include "../include/battery_charge_simulation.hpp"


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Battery_Charge_Simulation::Battery_Charge_Simulation(){
}

Battery_Charge_Simulation::~Battery_Charge_Simulation() {

}


int main(int argc, char **argv){
  ros::init(argc, argv, "battery_charge_simulation");
  ros::NodeHandle nh("~");

  ros::spin();
  return 0;
}
