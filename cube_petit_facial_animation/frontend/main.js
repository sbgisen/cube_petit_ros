class Ros {
  constructor () {
    this.ros = null
    this.expression = new CubeExpression()
    this.speech = new CubeSpeech()
    this.emoji = new CubeEmoji()
  }

  connect () {
    const ws = 'ws://localhost:9090/'
    this.ros = new ROSLIB.Ros({ url: ws })
    this.ros.on('close', () => {
      console.log('closed.')
      this.ros = null
    })
    this.expression.connect(this.ros)
    this.speech.connect(this.ros)
    this.emoji.connect(this.ros)
    console.log('connected.')
  }
}

class CubeExpression {
  constructor () {
    this.currentStatus = null // 表示中のステータス
    this.nextStatus = 'normal' // 表示待ちのステータス
  }

  connect (ros) {
    const orderSub = new ROSLIB.Topic({
      ros,
      name: '/face_node/set_expression',
      messageType: 'cube_expression/FaceExpression'
    })
    orderSub.subscribe(message => {
      const data = message.expression
      if (emotes[data] === undefined) {
        console.error('emote:' + data + ' is undefined.')
        return
      }
      console.log('emote:' + data)
      this.nextStatus = data
    })
  }
}

class CubeSpeech {
  constructor () {
    this.currentStatus = false // 表示中のステータス
    this.nextStatus = false // 表示待ちのステータス
  }

  connect (ros) {
    this.startSub = new ROSLIB.Topic({
      ros,
      name: '/speech_server/goal',
      messageType: 'cube_speech/SpeechActionGoal'
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

class CubeEmoji {
  constructor () {
    this.currentStatus = 'none' // 表示中のステータス
    this.nextStatus = 'none' // 表示待ちのステータス
  }

  connect (ros) {
    const emojiSub = new ROSLIB.Topic({
      ros,
      name: '/face_node/set_emoji',
      messageType: 'std_msgs/String'
    })
    emojiSub.subscribe(e => {
      const data = e.data
      if (emojis[data] === undefined) {
        console.error('emoji:' + data + ' is undefined.')
        return
      }
      console.log('emoji:' + data)
      this.nextStatus = data
    })
  }
}

/**
 * main
 */
const emotes = { normal, happy, sad, puzzled, angry }
let lastConnect = 0 // 前回の接続時間
const rosbridge = new Ros()
rosbridge.connect()
requestAnimationFrame(loop) // ループ処理を開始

async function loop (ts) {
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
  // 絵文字を更新
  const j = rosbridge.emoji
  if (j.currentStatus !== j.nextStatus) {
    emojis[j.currentStatus].stop()
    emojis[j.nextStatus].start()
    j.currentStatus = j.nextStatus
  }
  requestAnimationFrame(loop)
}
