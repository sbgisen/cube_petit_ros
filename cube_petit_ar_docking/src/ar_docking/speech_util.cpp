#include "speech_util.hpp"


int Speech_Util::sayText(std::string text){
  ROS_INFO("Speech_Util::say");
  speech_service.request.speech_text = text;
  speech_service.request.speech_method = type_default_;
  speech_service.request.emotion = emotion_default_;
  speech_service.request.emotion_level = emo_level_default_;
  speech_service.request.pitch = pitch_default_;
  speech_service.request.speed = speed_default_;
 
  if (client.call(speech_service)){
    ROS_INFO("Sum: %d", (int)speech_service.response.result);
  }else{
    ROS_WARN("Failed to call service");
    return 1;
  }
  return 0;
}

int Speech_Util::say(std::string text, std::string type, std::string emotion, int emo_level, int pitch, int speed) {
  ROS_INFO("Speech_Util::say");
  speech_service.request.speech_text = text;
  speech_service.request.speech_method = type;
  speech_service.request.emotion = emotion;
  speech_service.request.emotion_level = emo_level;
  speech_service.request.pitch = pitch;
  speech_service.request.speed = speed;
  if (client.call(speech_service)){
    ROS_INFO("Sum: %d", (int)speech_service.response.result);
  }else{
    ROS_WARN("Failed to call service");
    return 1;
  }
  return 0;
}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Speech_Util::Speech_Util(ros::NodeHandle nh)
  //speech_service(nh, "speech_service")//, boost::bind(&AR_Docking_Server::actionServerCallback, this, _1, &server), false),
 {
  ROS_INFO("Speech_Util::Speech_Util");

  type_default_ = "jtalk";  // jtalk or voicetext
  emotion_default_ = "happiness"; // happiness anger sadnesss shout default (None = happiness)
  emo_level_default_ = 2; // 1,2,3 or 4 (None is 2)  only voicetext
  pitch_default_  = 100;  // voicetext 50~200 , jtalk 0~300
  speed_default_ = 100;      // voicetext 50~400, jtalk 0~300

 client = nh.serviceClient<cube_petit_ar_docking::CubeSpeech>("/speech_server");

}

Speech_Util::~Speech_Util() {
  ROS_INFO("Speech_Util::Speech_Util");
}

