/**
 * 各表情共通のUtil
 * WARNING:同じtargetsのアニメは一度に一つまで
 */

/**
 * 指定方向を見る
 * 半径(0~1)とオイラー角で指定
 */
async function lookAt (r, euler) {
  // 各パーツの可動域
  const eyeRangeX = 12
  const eyeRangeY = 10
  const faceRangeX = 60
  const faceRangeY = 44
  const x = Math.cos(euler) * r
  const y = Math.sin(euler) * r
  const options = {
    duration: 2000,
    easing: 'easeInOutBack'
  }
  return Promise.all([
    // 顔の移動
    anime({
      targets: '.faceBox svg',
      translateX: x * faceRangeX,
      translateY: y * faceRangeY,
      ...options,
      delay: 100
    }).finished,
    // 目の移動
    anime({
      targets: '.right-eye, .left-eye',
      translateX: x * eyeRangeX,
      translateY: y * eyeRangeY,
      ...options
    }).finished
  ])
}

/** 瞬きループ */
const blink = {
  d: 8000, // 全体のduration
  targets: '.right-eye, .left-eye',
  anime: null,
  start () {
    const blinkKeyFrame = [
      { scaleY: 0.05, duration: 0.015 * this.d },
      { scaleY: 1, duration: 0.015 * this.d }
    ]
    this.anime = anime({
      targets: this.targets,
      loop: true,
      easing: 'linear',
      keyframes: [
        ...blinkKeyFrame,
        { scaleY: 1, duration: 0.455 * this.d },
        ...blinkKeyFrame,
        ...blinkKeyFrame,
        { scaleY: 1, duration: 0.455 * this.d }
      ]
    })
  },
  async stop () {
    if (this.anime) {
      this.anime.remove(this.targets)
      this.anime = null
    }
    return anime({
      targets: this.targets,
      scaleY: 1,
      duration: 0.015 * 8000,
      easing: 'linear'
    }).finished
  }
}

/** 顔パーツのアニメおよびcssをリセット */
function resetFace () {
  const targets = '.faceBox svg, .faceBox svg *'
  anime.remove(targets)
  $(targets).css({ transform: '' })
}

/** 表情をフェードイン */
function emoteFadeIn (selector) {
  // 対象を最後面に、フェードアウトを待つ
  $(selector).css('z-index', 0)
  $(selector).show()
}

/** 表情をフェードアウト */
function emoteFadeOut (selector) {
  // 対象を最前面にした後フェードアウト
  $(selector).css('z-index', 1)
  $(selector).fadeOut('slow')
}

/** スリープ */
function sleep (timeout) {
  return new Promise((resolve) => setTimeout(() => resolve(), timeout))
}

/** direction: alternateのループ終了を待つ */
function waitAlternateLoopComplete(anim){
  return new Promise(async (resolve) => {
    let stopped = false
    anim.loopComplete = () => {
      // 2回目のループ終了を待つ
      if (anim.reversed) stopped = true
    }
    while(!stopped){
      await sleep(100)
    }
    resolve()
  })
}
