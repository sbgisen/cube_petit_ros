## Juliusのセットアップ方法

```
cd setup
source install-julius.sh
```

## Cube-petit(固有名詞)を辞書に追加する

Cuboidくん、cube_petitを認識できるようにするため、
辞書ファイル`csj.pdp.htkdic`に読み方と発音を登録します

```
cd ~/lib/julius_libs/dictation-kit/model/lang_m/

vim bccwj.60k.pdp.htkdic
```

`bccwj.60k.pdp.htkdic`の一番下に２行追加
```
キューボイド  [キューボイド]    ky_B u:_E b_I o_I i_E d_I o_E
キューブプチ  [キューブプチ]    ky_B u:_I b_I u_E p_B u_I ch_I i_E
```
