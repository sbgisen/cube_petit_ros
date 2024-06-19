const sad = {
  tl: null,
  start () {
    emoteFadeIn('.sad')
    this.tl = anime.timeline({ loop: true })
    this.tl.add({
      targets: '.tear',
      scaleY: [{
        value: [0, 1],
        duration: 200,
        delay: 200,
        easing: 'linear'
      }],
      translateY: [
        { value: 800, duration: 1800, easing: 'cubicBezier(1,0,0.91,.19)' },
        { value: 800, duration: 1000 }
      ],
      loop: true
    })
    this.tl.add({
      targets: '.nose',
      rotate: [
        { value: -7, duration: 60, easing: 'linear' },
        { value: 4, duration: 120, easing: 'linear' },
        { value: 0, duration: 2000 }
      ],
      loop: true
    }, 2000)
  },
  stop () {
    this.tl = null
    resetFace()
    emoteFadeOut('.sad')
  },
  speech: {
    anime: null,
    defaultPath: null,
    targets: '.sad .mouth path',
    init(){
      this.defaultPath = $(this.targets).attr('d')
      this.anime = anime({
        targets: this.targets,
        d: [
          this.defaultPath,
          'm 493.32707,533.55081 c -28.80227,-1.37939 -67.22353,-1.10877 -85.67583,-11.95471 -13.94327,-18.21939 13.11047,-42.38571 37.05595,-56.09802 23.94548,-13.71231 33.52697,-21.47053 56.73232,-20.96291 23.20535,0.50762 32.4181,4.29021 57.69432,25.39598 13.47068,11.24808 49.85159,52.77158 26.88896,59.63792 -28.287,4.39277 -61.80406,5.4612 -92.69572,3.98174 z'
        ],
        direction: 'alternate',
        easing: 'steps(2)',
        duration: 200,
        loop: true
      })
    },
    start () {
      if (!this.anime){
        this.init()
        console.log('speech animation started')
      } else {
        this.anime.restart()
        console.log('speech animation restarted')
      }
    },
    async stop () {
      if (!this.anime) {
        console.log('speech animation not initialized')
      } else {
        await waitAlternateLoopComplete(this.anime)
        this.anime.pause()
        await sleep(100)
        $(this.targets).attr('d', this.defaultPath)
        console.log('speech animation stopped')
      }
    }
  }
}
