const surprised = {
  timer: null,
  start () {
    emoteFadeIn('.surprised')
    // びっくりして跳ねる
    anime({
      targets: '.surprised svg',
      translateY: [
        { value: -30, duration: 180, easing: 'easeOutQuad' },
        { value: 0, duration: 700, easing: 'spring(1, 60, 8, 0)' }
      ]
    })
    // 目を見開く
    anime({
      targets: '.surprised .right-eye, .surprised .left-eye',
      scale: [
        { value: 1.35, duration: 180, easing: 'easeOutQuad' },
        { value: 1.15, duration: 500, easing: 'easeOutElastic(1, .5)' }
      ]
    })
    // 口をまるくすぼめる
    anime({
      targets: '.surprised .mouth',
      scale: [
        { value: 0.7, duration: 180, easing: 'easeOutQuad' },
        { value: 1, duration: 600, easing: 'easeOutElastic(1, .5)' }
      ]
    })
    // 落ち着いたらそっと浮遊
    this.timer = setTimeout(() => { breath.start('.surprised', 6, 2200) }, 900)
  },
  stop () {
    clearTimeout(this.timer)
    resetFace()
    emoteFadeOut('.surprised')
  }
}
