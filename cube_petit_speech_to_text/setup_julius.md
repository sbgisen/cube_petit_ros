# setup_julius.md
日本語の音声認識ソフトJuliusの導入方法を説明します

## Manual Install

1. https://so-zou.jp/software/tech/library/julius/introduction/ からJuliusをダウンロードする
2. https://so-zou.jp/software/ontan/usage/ から話し言葉モデルキット([ssr-kit-v4.5.zip])をダウンロードする


3. `~/lib` に移動させて解凍する

```
mkdir ~/lib
cd lib
tar -zxvf julius-4.4.2.tar.gz 
rm julius-4.4.2.tar.gz ssr-kit-v4.5.zip 
```

4. Juliusのコンパイル

```
cd ../julius-4.4.2/
sudo apt install build-essential zlib1g-dev libsdl2-dev
sudo apt install libasound2-dev
./configure 
make
sudo checkinstall
```

## Exec
```
cd ~/lib/ssr-kit-v4.5 
source run-linux.sh
```


## Add Dictionary
Cuboidくん、cube_petitを認識できるようにするため、
辞書ファイル`csj.pdp.htkdic`に読み方と発音を登録します
```
cd ~/lib/ssr-kit-v4.5/models/
dconf write /org/gnome/gedit/preferences/encodings/candidate-encodings "['CURRENT', 'SHIFT_JIS', 'UTF-8', 'UTF-16']"
gedit csj.pdp.htkdic 
```

csj.pdp.htkdicの一番下に２行追加
```
キューボイド  [キューボイド]    ky_B u:_E b_I o_I i_E d_I o_E
キューブプチ  [キューブプチ]    ky_B u:_I b_I u_E p_B u_I ch_I i_E
```

結果
```
pass1_best:  キューブプチ 
pass1_best_wordseq: <sil> キューブプチ <sp>
pass1_best_phonemeseq: sp_S | ky_B u:_I b_I u_E p_B u_I ch_I i_E | sp_S
pass1_best_score: 50.793644
### Recognition: 2nd pass (RL heuristic best-first)
STAT: 00 _default: 1894 generated, 924 pushed, 49 nodes popped in 105
sentence1:  キューブプチ 
wseq1: <sil> キューブプチ <sil>
phseq1: sp_S | ky_B u:_I b_I u_E p_B u_I ch_I i_E | sp_S
cmscore1: 0.990 0.757 1.000
score1: 66.775040

pass1_best:  キューボイド 
pass1_best_wordseq: <sil> キューボイド <sp>
pass1_best_phonemeseq: sp_S | ky_B u:_E b_I o_I i_E d_I o_E | sp_S
pass1_best_score: 129.765869
### Recognition: 2nd pass (RL heuristic best-first)
STAT: 00 _default: 3848 generated, 1101 pushed, 96 nodes popped in 96
sentence1:  キューボイド 
wseq1: <sil> キューボイド <sil>
phseq1: sp_S | ky_B u:_E b_I o_I i_E d_I o_E | sp_S
cmscore1: 0.544 0.849 1.000
score1: 120.815346
```