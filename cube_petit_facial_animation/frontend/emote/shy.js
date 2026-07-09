const shy = {
  start () {
    emoteFadeIn('.shy')
    // ちょっとうつむいて、もじもじ
    anime({
      targets: '.shy svg',
      translateY: [0, 16],
      rotate: [-2, 2],
      duration: 2000,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
    // ほっぺがぽっぽする
    anime({
      targets: '.shy .blush',
      opacity: [0.5, 1],
      duration: 900,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutSine'
    })
  },
  stop () {
    resetFace()
    emoteFadeOut('.shy')
  }
}
