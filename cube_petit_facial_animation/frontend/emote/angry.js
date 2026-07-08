const angry = {
  start () {
    emoteFadeIn('.angry')
    // ぷんぷん(ときどき小さくぷるぷるする)
    anime({
      targets: '.angry svg',
      loop: true,
      keyframes: [
        { translateX: -5, duration: 60, easing: 'linear' },
        { translateX: 4, duration: 60, easing: 'linear' },
        { translateX: -3, duration: 60, easing: 'linear' },
        { translateX: 2, duration: 60, easing: 'linear' },
        { translateX: 0, duration: 60, easing: 'linear' },
        { translateX: 0, duration: 2400, easing: 'linear' }
      ]
    })
    blink.start()
  },
  stop () {
    resetFace()
    emoteFadeOut('.angry')
  },
  speech: makeSpeech(
    '.angry .mouth path',
    'm 493.32707,533.55081 c -28.80227,-1.37939 -67.22353,-1.10877 -85.67583,-11.95471 -13.94327,-18.21939 13.11047,-42.38571 37.05595,-56.09802 23.94548,-13.71231 33.52697,-21.47053 56.73232,-20.96291 23.20535,0.50762 32.4181,4.29021 57.69432,25.39598 13.47068,11.24808 49.85159,52.77158 26.88896,59.63792 -28.287,4.39277 -61.80406,5.4612 -92.69572,3.98174 z'
  )
}
