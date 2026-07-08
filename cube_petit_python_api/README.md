# cube_petit_python_api

ROS を知らなくても cube_petit を Python から動かせるパッケージです。
`rclpy.init()` も `spin()` も不要。3行でしゃべります。

```python
from cube_petit_python_api import CubePetit

robot = CubePetit()          # デフォルトは cube_petit_orange
robot.say('こんにちは')       # しゃべり終わるまで待って返ってくる
```

裏側では専用ノード + MultiThreadedExecutor がデーモンスレッドで spin しているので、
利用者が ROS のイベントループを意識する必要はありません。

## セットアップ

ロボット側で cube_petit のスタック(bringup / navigation / facial_animation)が起動していれば、
同じ ROS ネットワーク上の Python からそのまま使えます。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros/install/setup.bash
python3
```

## CubePetit の使い方

```python
from cube_petit_python_api import CubePetit

with CubePetit(robot='cube_petit_orange', timeout=5.0) as robot:
    robot.say('おはよう', emotion='happy')       # 発話(同期)
    robot.set_face('happy')                      # 表情変更
    pose = robot.where_am_i()                    # 地図上の現在位置 (x, y, yaw)
    robot.move_to('favorite')                    # お気に入りの場所へ移動(非同期)
    robot.move_to((1.0, 2.0, 0.0), wait=True)    # 座標指定で移動完了まで待つ
    robot.cancel_move()                          # 移動・パトロール中止
    robot.remember_place('kitchen')              # 現在位置を場所として保存
```

### メソッド一覧

| メソッド | 説明 |
| --- | --- |
| `say(text, emotion='normal', wait=True, timeout=None)` | 発話。emotion は `normal` / `happy` / `angry` / `sad` / `shout` |
| `set_face(expression='normal', timeout=None)` | 表情変更。`normal` / `happy` / `angry` / `sad` / `puzzled` |
| `move_to(target, wait=False, timeout=None, arrival_timeout=300)` | `'favorite'` / `'patrol'` / `(x, y, yaw)` へ移動 |
| `cancel_move(timeout=None)` | 移動・パトロールのキャンセル |
| `where_am_i(timeout=None)` | 地図上の現在位置 `RobotPose(x, y, yaw)`(TF `map`→`base_link`) |
| `remember_place(name, category='place', timeout=None)` | 現在位置を場所として保存 |
| `close()` | ノード・スレッドの後始末(`with` 文なら自動) |

### 接続先の指定

- コンストラクタ引数: `CubePetit(robot='cube_petit_pink')`
- 環境変数: `PETIT_ROBOT_NS=cube_petit_pink`(引数が優先)
- `domain_id=` で ROS_DOMAIN_ID も指定可能

### エラー

ロボットが起動していない・届かないときは、タイムアウト後に
`CubePetitNotRunning`(「ロボットが起動していないみたい…」)が上がります。
永遠に固まることはありません。引数がおかしいときは `ValueError`、
`close()` 後の呼び出しは `CubePetitClosed` です。

### スレッドと寿命

- 1 プロセス内で作成・`close()` を繰り返して OK(インスタンスごとに独立した rclpy コンテキストを持ちます)
- 各メソッドは複数スレッドから呼んでも安全ですが、`close()` との競合だけは避けてください

## Python すら使わない場合

HTTP で叩ける Web API(`web_interface`)もあります。ROS 環境ゼロのマシンからは
そちらが便利です。

## 上級者向け(従来のコマンダー)

既存の `CubePetitCommander` / `SpeechCommander` / `GPTChatCommander` はそのまま残っています。
自分でノードを管理したい場合はこちらを使ってください。

```python
import rclpy
from rclpy.node import Node
from cube_petit_python_api.commanders.speech import SpeechCommander

rclpy.init()
node = Node('test')
speech = SpeechCommander(node)
speech.say('test')
```

GPT チャット(要 `OPENAI_API_KEY`):

```python
from cube_petit_python_api.commanders.gpt_chat import GPTChatCommander
```

## テスト

```bash
# ROS 環境なしでも動く純ロジックテスト + ROS 環境があればライフサイクルテスト
python3 -m pytest cube_petit_python_api/test -v
```
