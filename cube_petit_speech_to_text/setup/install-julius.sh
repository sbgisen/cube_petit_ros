#! /bin/sh

# juliusでディクテーションを行うために必要なライブラリのインストールスクリプトです。
# Ubuntuを想定し、checkinstallコマンドを使用してインストールしています。

# セットアップに必要な機能をインストール
sudo apt install -y autotools-dev autoconf software-properties-common
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt update
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
git clone https://github.com/julius-speech/julius.git
cd julius
./configure
make
sudo checkinstall -y --pkgname='julius' --pkgversion='4.5'

# dictation-kitのダウンロード(検証済み：v4.3.1)
cd ..
git clone https://github.com/julius-speech/dictation-kit.git

cd $now_dir