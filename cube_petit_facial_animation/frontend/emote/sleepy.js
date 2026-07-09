const sleepy = {
  start () {
    emoteFadeIn('.sleepy')
    // 半目でとろとろ、ときどき目を閉じる
    anime({
      targets: '.sleepy .right-eye, .sleepy .left-eye',
      loop: true,
      keyframes: [
        { scaleY: 0.45, duration: 600, easing: 'easeOutQuad' },
        { scaleY: 0.45, duration: 2600, easing: 'linear' },
        { scaleY: 0.06, duration: 1000, easing: 'easeInQuad' },
        { scaleY: 0.06, duration: 2200, easing: 'linear' },
        { scaleY: 0.45, duration: 800, easing: 'easeOutQuad' }
      ]
    })
    // こっくりこっくり
    anime({
      targets: '.sleepy svg',
      translateY: [0, 26],
      rotate: [0, 2.5],
      duration: 3600,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
    // Zzzがふわふわ浮かぶ
    anime({
      targets: '.sleepy .zzz',
      loop: true,
      opacity: [
        { value: [0, 0.9], duration: 900 },
        { value: 0, duration: 900, delay: 1400 }
      ],
      translateY: [{ value: [20, -30], duration: 3200 }],
      delay: anime.stagger(700),
      easing: 'easeInOutSine'
    })
  },
  stop () {
    resetFace()
    emoteFadeOut('.sleepy')
  }
}
