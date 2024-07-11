#! /bin/sh -e

# juliusでディクテーションを行うために必要なライブラリのインストールスクリプトです。
# Ubuntuを想定し、checkinstallコマンドを使用してインストールしています。

# セットアップに必要な機能をインストール
sudo apt-get install -y autotools-dev autoconf software-properties-common
sudo apt-get install -y portaudio19-dev
sudo apt-get install -y cpanminus
sudo cpanm Jcode
sudo apt-get install -y libsndfile1-dev
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt-get update
# git LFSをインストール(dictation-kitの大サイズのファイルをcloneする際に必要)
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install

# juliusのインストール
# (参考:http://motok5.hatenablog.com/entry/2018/09/22/083226)
now_dir=$PWD
mkdir -p ~/lib/julius_libs
cd ~/lib/julius_libs

# julius本体のダウンロード(検証済み：v4.5)

if [ -f "~/lib/julius_libs/julius" ]; then
  echo "already installed julius"
else
  git clone https://github.com/julius-speech/julius.git
  cd julius
  ./configure
  make
  sudo apt-get install -y checkinstall
  sudo checkinstall -y --pkgname='julius' --pkgversion='4.5'
  cd ..
fi

# dictation-kitのダウンロード(検証済み：v4.3.1)
if [ -f "~/lib/julius_libs/dictation-kit" ]; then
  echo "already installed dictation-kit"
else
  git clone https://github.com/julius-speech/dictation-kit.git
  cd ~/lib/julius_libs/dictation-kit/model/lang_m/
  if [ -f "bccwj.60k.pdp.htkdic" ]; then
    echo "キューボイド  [キューボイド]    ky_B u:_E b_I o_I i_E d_I o_E" >>  bccwj.60k.pdp.htkdic
    echo "キューブプチ  [キューブプチ]    ky_B u:_I b_I u_E p_B u_I ch_I i_E" >>  bccwj.60k.pdp.htkdic
  fi
fi

cd $now_dir
