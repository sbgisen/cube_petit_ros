#include "cube_petit_hardware_interface.hpp"



/* -----------------------------

dji_control.cpp


関数

コンストラクタ
デコンストラクタ
READ関数
WRITE関数

------------------------------- */


std::string execCmd(std::string system_cmd){ //static
  FILE *fp;
//  ROS_INFO("[%s]" system_cmd);
  std::string result = "";
  char buf[256];
  if( (fp = popen(system_cmd.c_str(),"r")) != NULL )
  {
    fgets(buf, 256, fp);
    pclose(fp);
    result = buf;
  }
  return result;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dji_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    
    // CAN通信のセットをする
    int count =1;
    std::string result;
    result = execCmd("echo gisen55 | sudo -S bash ~/ros/src/sbgisen/raspberrypi_cube_moc/cube_petit_hardware_interface/setCan0.sh");
    //結果は success can0
    // failed 1 or 2 or 3
    std::cout << "resu is:" << result << std::endl;   
    //ROS_INFO("resu is: %s", result);

    
    while(ros::ok()){
        ROS_INFO("hello");
        ros::spinOnce();
        loop_rate.sleep();
        count++;
        if(count >= 5){
            break;
        }
    }



    return 0;
}

void Cube_Petit_Hardware_Interface::read(){
}

void Cube_Petit_Hardware_Interface::write(){
}


Cube_Petit_Hardware_Interface::Cube_Petit_Hardware_Interface(){
}

Cube_Petit_Hardware_Interface::~Cube_Petit_Hardware_Interface(){
}