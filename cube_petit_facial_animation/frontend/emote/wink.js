const wink = {
  timer: null,
  start () {
    emoteFadeIn('.wink')
    breath.start('.wink', 8, 2400)
    this.loop()
  },
  loop () {
    // 左目でぱちり
    anime({
      targets: '.wink .left-eye',
      keyframes: [
        { scaleY: 0.08, duration: 140, easing: 'easeInQuad' },
        { scaleY: 0.08, duration: 800, easing: 'linear' },
        { scaleY: 1, duration: 200, easing: 'easeOutQuad' }
      ]
    })
    // ちょっと首をかしげる
    anime({
      targets: '.wink svg',
      keyframes: [
        { rotate: -4, duration: 200, easing: 'easeOutQuad' },
        { rotate: -4, duration: 700, easing: 'linear' },
        { rotate: 0, duration: 260, easing: 'easeOutQuad' }
      ]
    })
    this.timer = setTimeout(() => { this.loop() }, anime.random(2600, 4200))
  },
  stop () {
    clearTimeout(this.timer)
    resetFace()
    emoteFadeOut('.wink')
  },
  speech: makeSpeech(
    '.wink .mouth path',
    'm 493.37466,540.44539 c -14.18298,-1.97172 -27.48143,-10.00968 -38.37935,-19.29839 -10.01059,-8.53241 -19.22735,-19.42677 -23.08844,-32.0008 -2.64872,-8.62582 -1.47698,-18.3243 0.78894,-27.05849 2.88653,-11.12641 8.01919,-22.22306 15.87883,-30.61086 4.12889,-4.40635 9.90029,-7.12105 15.50504,-9.36837 11.51478,-4.61707 23.97903,-6.89931 36.32752,-8.09187 7.94959,-0.76774 16.00718,-0.18533 23.95125,0.63763 7.00858,0.72605 14.15557,1.42856 20.79737,3.78096 8.121,2.8763 16.88969,6.02455 22.67013,12.41283 9.08583,10.04126 13.07991,24.14763 15.86264,37.40039 2.0291,9.66356 4.1744,20.50464 0.37918,29.62045 -5.78418,13.89315 -19.0905,24.45547 -32.44378,31.39565 -17.54297,9.11772 -38.66676,13.90324 -58.24933,11.18087 z'
  )
}
