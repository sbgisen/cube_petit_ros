#!/bin/bash

KILL_FLAG=0
CMD=`ps aux |grep -v grep| grep speech_server.launch`

if [ "${CMD}" == "" ]; then
    KILL_FLAG=1
    echo "speech_serverを起動します。"
    roslaunch cube_speech speech_server.launch &
fi

# voltage
VOLTAGE=`roslaunch jbd_battery_monitor battery_monitor.launch once_flag:=true |grep Voltage | tail -n 1 | awk '{print $5}'`
# current
CURRENT=`roslaunch jbd_battery_monitor battery_monitor.launch once_flag:=true |grep Current |  tail -n 1 | awk '{print $5}'`
# remain
REMAIN=`roslaunch jbd_battery_monitor battery_monitor.launch once_flag:=true |grep Remain |  tail -n 1 | awk '{print $5}'`

echo "バッテリの状態をお知らせします。"
TALK="バッテリの状態をお知らせします。"

echo "バッテリ電圧は"${VOLTAGE}
echo "電流値は"${CURRENT}
echo "残量は"${REMAIN}

TALK1=`echo "バッテリ電圧は${VOLTAGE}" | cut -d "." -f 1`
TALK2=`echo "電流値は${CURRENT}" | cut -d "." -f 1`
TALK3=`echo "残量は${REMAIN}" | cut -d "." -f 1`
TALK11=`echo "${VOLTAGE}" | cut -d "." -f 2`
TALK22=`echo "${CURRENT}" | cut -d "." -f 2`
TALK33=`echo "${REMAIN}" | cut -d "." -f 2`

rosservice call /speech_server "{speech_text: '${TALK}', speech_method: 'jtalk', emotion: 'happiness', emotion_level: 2, pitch: 150, speed: 100}"
rosservice call /speech_server "{speech_text: '${TALK1}てん${TALK11:0:2}。${TALK2}てん${TALK22:0:2}。${TALK3}てん${TALK33:0:2}', speech_method: 'jtalk', emotion: 'happiness', emotion_level: 2, pitch: 150, speed: 100}"


echo "---------------------"

result=`echo "${CURRENT}" | cut -d "." -f 1`
if [ $(($result)) -gt 0 ]; then
    echo "現在充電中です。"
    TALK="現在充電中です。"

    result1=`echo "${REMAIN}" | cut -d "." -f 1`
    if [ $(($result1)) -gt 9 ]; then
        echo "充電が完了しました。"
        TALK="充電が完了しました。"

    fi
    rosservice call /speech_server "{speech_text: '${TALK}', speech_method: 'jtalk', emotion: 'happiness', emotion_level: 2, pitch: 150, speed: 100}"
fi

result2=`echo "${REMAIN}" | cut -d "." -f 1`
if [ $(($result)) -lt 0 -a $(($result2)) -lt 2 ]; then
    echo "もうすぐ電池が切れます。充電してください。"
    TALK="もうすぐ電池が切れます。充電してください。"
    rosservice call /speech_server "{speech_text: '${TALK}', speech_method: 'jtalk', emotion: 'happiness', emotion_level: 2, pitch: 150, speed: 100}"
fi

if [ ${KILL_FLAG} -eq 1 ]; then
    echo "speech_serverを終了します。"
    bash /home/gisen/shell_scripts/ps_grep_kill.sh speech_server.launch
fi