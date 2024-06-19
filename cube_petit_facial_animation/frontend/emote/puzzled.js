const puzzled = {
  start () {
    emoteFadeIn('.puzzled')
    // 下を向く
    anime({
      targets: '.puzzled svg',
      translateY: [20, 0],
      duration: 3000,
      direction: 'alternate',
      loop: true,
      easing: 'easeInOutBack'
    })
  },
  stop () {
    resetFace()
    emoteFadeOut('.puzzled')
  },
  speech: {
    anime: null,
    defaultPath: null,
    targets: '.puzzled .mouth path',
    init(){
      this.defaultPath = $(this.targets).attr('d')
      this.anime = anime({
        targets: this.targets,
        d: [
          this.defaultPath,
          'm 432.39171,496.07521 c 0.45857,-13.64574 4.53495,-28.11519 12.96728,-38.85353 10.7115,-13.6408 27.13514,-23.83376 44.04101,-27.70666 19.01132,-4.35523 40.0343,-0.73531 58.15036,6.49012 16.11206,6.42615 33.74438,15.88598 41.49447,31.40468 6.51341,13.0424 7.13497,30.9393 0.14907,43.73484 -7.04707,12.90757 -23.03175,19.76313 -37.19269,23.72977 -12.43829,3.48411 -25.8287,-0.53035 -38.74203,-0.84024 -13.98933,-0.33571 -28.02186,0.074 -41.96554,-1.104 -10.21157,-0.86274 -22.79032,2.46771 -30.43322,-4.35919 -8.34827,-7.45694 -8.84466,-21.30838 -8.46871,-32.49579 z'
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
