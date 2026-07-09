/**
 * 各表情共通のUtil
 * WARNING:同じtargetsのアニメは一度に一つまで
 */

// ---- ありさん調整用 --------------------------------------------------
// 目全体を下げる量(px, viewBox座標系。+で下に下がる)
const EYE_Y_OFFSET = 16
// 目の傾き(deg)。左目に+、右目に-で左右対称に適用される。0で無効
const EYE_TILT_DEG = 0
// ----------------------------------------------------------------------

// 目(ハート・ぐるぐる目も含む)を位置調整用の<g class="eye-pos">で包む。
// アニメはg.right-eye等を直接動かすため、このラッパの調整値は上書きされない
$('g.right-eye, g.left-eye, g.heart, g.spiral').each(function () {
  const d = $(this).find('path').attr('d') || ''
  const x = parseFloat((d.match(/-?[\d.]+/) || [0])[0])
  const side = x >= 512 ? 'right' : 'left'
  const wrapper = document.createElementNS('http://www.w3.org/2000/svg', 'g')
  wrapper.setAttribute('class', 'eye-pos eye-pos-' + side)
  this.parentNode.insertBefore(wrapper, this)
  wrapper.appendChild(this)
})
document.documentElement.style.setProperty('--eye-y-offset', EYE_Y_OFFSET + 'px')
document.documentElement.style.setProperty('--eye-tilt', EYE_TILT_DEG + 'deg')

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
      targets: 'svg',
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

/**
 * 瞬きループ
 * ランダムな間隔で瞬きし、たまに2連瞬きする
 */
const blink = {
  targets: '.right-eye, .left-eye',
  anime: null,
  timer: null,
  start () {
    this.schedule(anime.random(500, 2500))
  },
  schedule (wait) {
    this.timer = setTimeout(() => { this.once() }, wait)
  },
  once () {
    // 25%の確率で2連瞬き
    const twice = anime.random(1, 100) <= 25
    const blinkKeyFrame = [
      { scaleY: 0.05, duration: 70 },
      { scaleY: 1, duration: 110 }
    ]
    this.anime = anime({
      targets: this.targets,
      easing: 'linear',
      keyframes: twice
        ? [...blinkKeyFrame, { scaleY: 1, duration: 130 }, ...blinkKeyFrame]
        : blinkKeyFrame
    })
    this.schedule(anime.random(2000, 6500))
  },
  /** 次回予約の取り消しのみ(表情切り替え時にresetFaceから呼ばれる) */
  cancel () {
    clearTimeout(this.timer)
    this.timer = null
  },
  async stop () {
    this.cancel()
    if (this.anime) {
      this.anime.remove(this.targets)
      this.anime = null
    }
    return anime({
      targets: this.targets,
      scaleY: 1,
      duration: 120,
      easing: 'linear'
    }).finished
  }
}

/**
 * 呼吸のようなゆらゆら(浮遊感)
 * 各表情の .sway ラッパをゆっくり上下させる
 */
const breath = {
  targets: null,
  start (selector, range = 8, duration = 2800) {
    this.targets = selector + ' .sway'
    anime({
      targets: this.targets,
      translateY: [range / 2, -range / 2],
      duration,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
  },
  stop () {
    if (!this.targets) return
    anime.remove(this.targets)
    $(this.targets).css({ transform: '' })
    this.targets = null
  }
}

/** 顔パーツのアニメおよびcssをリセット */
function resetFace () {
  blink.cancel()
  breath.stop()
  const targets = '.sway, svg, svg *'
  anime.remove(targets)
  $(targets).css({ transform: '', opacity: '' })
}

/**
 * 発話時の口パクアニメを生成する(各表情共通)
 * @param {string} targets 口のpathセレクタ
 * @param {string} openPath 開いた口のパスデータ
 */
function makeSpeech (targets, openPath) {
  return {
    anime: null,
    defaultPath: null,
    targets,
    start () {
      this.defaultPath = $(this.targets).attr('d')
      this.anime = anime({
        targets: this.targets,
        d: [this.defaultPath, openPath],
        direction: 'alternate',
        easing: 'steps(2)',
        duration: 200,
        loop: true
      })
      console.log('speech animation started')
    },
    async stop () {
      if (!this.anime) return
      this.anime.loopComplete = async (anim) => {
        // direction: alternateのため、2回目のループまで待つ
        if (!anim.reversed) return
        anim.pause()
        await sleep(100)
        $(this.targets).attr('d', this.defaultPath)
        console.log('speech animation stopped')
      }
    }
  }
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
