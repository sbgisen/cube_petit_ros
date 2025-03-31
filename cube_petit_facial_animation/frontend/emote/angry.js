const angry = {
  start () {
    emoteFadeIn('.angry')
    // 跳ねるループ
    anime({
      targets: '.angry svg',
      translateY: [30, 0],
      duration: 200,
      direction: 'alternate',
      loop: true,
      easing: 'linear'
    })
    blink.start()
  },
  stop () {
    resetFace()
    emoteFadeOut('.angry')
  },
  speech: {
    anime: null,
    defaultPath: null,
    targets: '.angry .mouth path',
    init(){
      this.defaultPath = $(this.targets).attr('d')
      this.anime = anime({
        targets: this.targets,
        d: [
          this.defaultPath,
          'm377.79 505.64c-12.005-19.33 4.5853-46.142 19.475-54.759 30.919-17.893 62.73-20.841 93.875-24.902 37.556-4.897 94.805-4.9982 134.08 17.079 19.51 10.967 50.012 32.131 41.869 52.492-9.0369 22.596-36.645 26.441-55.568 28.525-28.305 3.1182-62.309 4.8974-92.607 12.039-27.301 6.4356-71.11 19.542-110.25-0.12917-19.423-9.7622-22.285-16.51-30.877-30.346z'
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
