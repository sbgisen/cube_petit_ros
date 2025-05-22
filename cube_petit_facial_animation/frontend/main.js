class Ros {
  constructor() {
    this.ros = null
    this.expression = new CubeExpression()
    this.speech = new CubeSpeech()
    this.gaze = new CubeGaze()
  }

  connect() {
    const ws = 'ws://localhost:9090/'
    this.ros = new ROSLIB.Ros({ url: ws })
    this.ros.on('close', () => {
      console.log('closed.')
      this.ros = null
    })
    this.expression.connect(this.ros)
    this.speech.connect(this.ros)
    this.gaze.connect(this.ros)
    console.log('connected.')
  }
}

class CubeExpression {
  constructor() {
    this.currentStatus = null // 表示中のステータス
    this.nextStatus = 'normal' // 表示待ちのステータス
  }

  connect(ros) {
    const orderSub = new ROSLIB.Topic({
      ros,
      name: '/facial_expression/expression_command',
      messageType: 'sbgisen_msgs/FaceExpression'
    })
    orderSub.subscribe(message => {
      const data = message.expression
      if (emotes[data] === undefined) {
        console.log('emote:' + data + ' is undefined.')
        return
      }
      console.log('emote:' + data)
      this.nextStatus = data
    })
  }
}

class CubeGaze {
  constructor() {
    this.lookResetTimer = null // タイマーを保持
  }

  connect(ros) {
    this.lookSub = new ROSLIB.Topic({
      ros,
      name: '/facial_expression/look_at',
      messageType: 'std_msgs/Float64MultiArray'
    })

    this.lookSub.subscribe(message => {
      const arr = message.data
      if (!Array.isArray(arr) || arr.length < 3) {
        console.warn('look_at: invalid format. Expected [radius, angle(deg), duration(sec)]')
        return
      }

      const [radius, angleDeg, durationSec] = arr
      const angleRad = angleDeg * Math.PI / 180

      console.log(`CubeGaze: lookAt(radius=${radius}, angle=${angleDeg}°, duration=${durationSec}s)`)

      // 視線を向ける
      lookAt(radius, angleRad)

      // 前のタイマーを解除
      if (this.lookResetTimer) {
        clearTimeout(this.lookResetTimer)
        this.lookResetTimer = null
      }

      // durationが0以外なら元に戻す
      if (durationSec > 0) {
        this.lookResetTimer = setTimeout(() => {
          lookAt(0, 0)
          console.log('CubeGaze: look reset to center')
        }, durationSec * 1000)
      }
    })
  }
}

class CubeSpeech {
  constructor() {
    this.currentStatus = false // 表示中のステータス
    this.nextStatus = false // 表示待ちのステータス
  }

  connect(ros) {
    this.startSub = new ROSLIB.Topic({
      ros,
      name: '/speech_server/goal',
      messageType: 'sbgisen_msgs/SpeechActionGoal'
    })
    this.endSub = new ROSLIB.Topic({
      ros,
      name: '/speech_server/result',
      messageType: 'cube_speech/SpeechActionResult'
    })
    this.startSub.subscribe(data => {
      this.nextStatus = true
      console.log('speech start: ' + data.goal.speech_text)
    })
    this.endSub.subscribe(data => {
      this.nextStatus = false
      console.log('speech end')
    })
  }
}

/**
 * main
 */
const emotes = { normal, happy, sad, puzzled }
let lastConnect = 0 // 前回の接続時間
const rosbridge = new Ros()
rosbridge.connect()
requestAnimationFrame(loop) // ループ処理を開始

async function loop(ts) {
  // console.log('loop')
  // 接続切れかつ前回の処理から1秒経過していたら再接続
  if (!rosbridge.ros && ts - lastConnect >= 1000) {
    console.log('connecting...')
    rosbridge.connect()
    lastConnect = ts
  }
  // 表情を更新
  const e = rosbridge.expression
  const s = rosbridge.speech
  if (e.currentStatus !== e.nextStatus &&
    s.currentStatus !== true) { // 発話中は更新しない
    // 初期状態のときは表情の停止はスキップ
    if (e.currentStatus !== null) {
      emotes[e.currentStatus].stop()
    }
    emotes[e.nextStatus].start()
    e.currentStatus = e.nextStatus
  }
  // 発話状態を更新
  const speech = emotes[e.currentStatus].speech
  if (s.currentStatus !== s.nextStatus) {
    // アニメーション未定義のときはステータスの更新のみ行う
    if (speech !== undefined) {
      if (s.nextStatus === true) speech.start()
      if (s.nextStatus === false) await speech.stop()
    }
    s.currentStatus = s.nextStatus
  }
  requestAnimationFrame(loop)
}
