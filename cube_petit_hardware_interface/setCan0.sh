#!/bin/sh

## SUDO

# You must set 'LOGFILE'
#readonly PROCNAME=${0##*/}
#function log() {
#  local fname=${BASH_SOURCE[1]##*/}
#  echo -e "$(date '+%Y-%m-%dT%H:%M:%S') ${PROCNAME} (${fname}:${BASH_LINENO[0]}:${FUNCNAME[1]}) $@"
#}

CAN_PORT="can0"
BIT_RATE="1000000"
if [ $# -gt 1 ]; then
  : # echo "[WARN] 指定された引数は1個です。" 1>&2
  : # echo "[INFO] デフォルトのポート名<"${CAN_PORT}">が指定されます" 1>&2
elif [ $# -eq  0 ]; then
  : # echo "[INFO] デフォルトのポート名<"${CAN_PORT}">が指定されます" 1>&2
else
  : # echo "[INFO] 指定されたポート名<"${CAN_PORT}">を使用します"
fi


# ifconfig ${CAN_PORT} $$ ip link set ${CAN_PORT} up type can bitrate ${BIT_RATE}

ifconfig ${CAN_PORT} > /dev/null 2>&1
if [ "$?" -ne 0 ]; then  
    echo "1"
    exit 1
fi

cmd=`ip link set ${CAN_PORT} up type can bitrate ${BIT_RATE} > /dev/null 2>&1`
if [ "${cmd}" != "0" ]; then
    : # nothing # echo "[WARN] Line2:"${CAN_PORT}"が忙しいか、存在しません"
fi

ip -details -statistics link show ${CAN_PORT} > /dev/null 2>&1
if [ "$?" -ne 0 ]; then
    echo "3"
    exit 1
fi

# echo "success "${CAN_PORT}""
echo "0"
exit 0
