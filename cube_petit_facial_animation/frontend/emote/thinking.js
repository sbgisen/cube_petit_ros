const thinking = {
  start () {
    emoteFadeIn('.thinking')
    // 目が上の方をゆっくり泳ぐ
    anime({
      targets: '.thinking .right-eye, .thinking .left-eye',
      loop: true,
      keyframes: [
        { translateX: 10, translateY: -10, duration: 1200, easing: 'easeInOutQuad' },
        { translateX: 10, translateY: -10, duration: 2200, easing: 'linear' },
        { translateX: -8, translateY: -12, duration: 1400, easing: 'easeInOutQuad' },
        { translateX: -8, translateY: -12, duration: 2000, easing: 'linear' },
        { translateX: 0, translateY: 0, duration: 1200, easing: 'easeInOutQuad' }
      ]
    })
    // 首をかしげる
    anime({
      targets: '.thinking svg',
      rotate: [0, -3],
      translateY: [0, 6],
      duration: 2600,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
    // 「…」が順番に浮かぶ
    anime({
      targets: '.thinking .dot',
      loop: true,
      opacity: [
        { value: [0, 1], duration: 400 },
        { value: 1, duration: 1200 },
        { value: 0, duration: 300 }
      ],
      delay: anime.stagger(500),
      easing: 'linear'
    })
  },
  stop () {
    resetFace()
    emoteFadeOut('.thinking')
  }
}
