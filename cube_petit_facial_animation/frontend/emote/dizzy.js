const dizzy = {
  start () {
    emoteFadeIn('.dizzy')
    // 目がぐるぐる
    anime({
      targets: '.dizzy .spiral',
      rotate: '1turn',
      duration: 1600,
      loop: true,
      easing: 'linear'
    })
    // ふらふら揺れる
    anime({
      targets: '.dizzy svg',
      translateX: [-14, 14],
      rotate: [-5, 5],
      duration: 1400,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
  },
  stop () {
    resetFace()
    emoteFadeOut('.dizzy')
  }
}
