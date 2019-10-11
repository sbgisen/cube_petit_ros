#include "ros/ros.h"
#include "geometry_msgs/Twist.h" //turtle_sim用
#include "sensor_msgs/Joy.h"    //joystick用

geometry_msgs::Twist cmd_vel; //turtle_simに送信するgeometry_msgs::Twist型変数

//joystickが押された時に代入する
void joy_callback(const sensor_msgs::Joy& joy_msg){
    cmd_vel.linear.x = joy_msg.axes[1];
    cmd_vel.angular.z = joy_msg.axes[0]*10;
}

int main(int argc, char **argv){
    ros::init(argc, argv , "cmd_vel_publisher");
    ros::NodeHandle n;

    //cmd_velというトピックにるgeometry_msgs::Twist型を送信する
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //joyというトピックにデータが来たらjoy_callbackを呼ぶ
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);

    //100がおおい
    //コントローラは30とか
    ros::Rate loop_rate(10);    //10Hz
    while(ros::ok){
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();        //publishされているかチェックする
        loop_rate.sleep();

    }
    return 0;
}