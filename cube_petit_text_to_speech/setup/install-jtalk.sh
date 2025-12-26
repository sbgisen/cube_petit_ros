#! /bin/sh -e

LIB_DIR=$(cd $(dirname $0); pwd)/speech_lib
if [ -f "$LIB_DIR/open_jtalk-1.11/bin/open_jtalk" ]; then
  exit 0
fi

mkdir -p $LIB_DIR

cd $LIB_DIR
# HTS Engine API のダウンロード&展開（検証済み：v1.10）
wget -O - 'http://downloads.sourceforge.net/hts-engine/hts_engine_API-1.10.tar.gz' | tar zxvf -
cd $LIB_DIR/hts_engine_API-1.10
./configure
make

cd $LIB_DIR
# openjtalk本体のダウンロード&展開（検証済み：v1.11）
wget  -O - 'http://downloads.sourceforge.net/open-jtalk/open_jtalk-1.11.tar.gz' | tar zxvf -
cd $LIB_DIR/open_jtalk-1.11
./configure \
--with-charset=utf-8 \
--with-hts-engine-header-path=$LIB_DIR/hts_engine_API-1.10/include \
--with-hts-engine-library-path=$LIB_DIR/hts_engine_API-1.10/lib
make

cd $LIB_DIR
# openjtalk辞書ファイルのダウンロード（検証済み：utf8 v1.11）
wget -O - 'https://sourceforge.net/projects/open-jtalk/files/Dictionary/open_jtalk_dic-1.11/open_jtalk_dic_utf_8-1.11.tar.gz/download?use_mirror=jaist' | tar zxvf -
# MMDAgentの音声ファイルダウンロード（検証済み：v1.6）
wget 'https://sourceforge.net/projects/mmdagent/files/MMDAgent_Example/MMDAgent_Example-1.6/MMDAgent_Example-1.6.zip'
unzip MMDAgent_Example-1.6.zip
rm MMDAgent_Example-1.6.zip

return 0
