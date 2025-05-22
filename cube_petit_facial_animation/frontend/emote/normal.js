const normal = {
  start () {
    emoteFadeIn('.normal')
    this.expire = Date.now()
    this.loop()
  },
  expire: null,
  timer: null,
  loop () {
    if (this.expire <= Date.now()) {
      // 瞬きを停止
      blink.stop().then(() => {
        // ランダムな方向を見る
        const r = anime.random(50, 100) / 100
        const euler = anime.random(0, 100) / 100 * Math.PI * 2
        lookAt(r, euler).then(() => {
          // 視線を戻す
          lookAt(0, 0).then(() => {
            // 瞬きを再開
            blink.start()
            this.expire = Date.now() + anime.random(5, 15) * 1000
            this.timer = setTimeout(() => { this.loop() }, 1000)
          })
        })
      })
    } else {
      this.timer = setTimeout(() => { this.loop() }, 1000)
    }
  },
  stop () {
    clearTimeout(this.timer)
    resetFace()
    emoteFadeOut('.normal')
  },
  speech: {
    anime: null,
    defaultPath: null,
    targets: '.normal .mouth path',
    start () {
      this.defaultPath = $(this.targets).attr('d')
      this.anime = anime({
        targets: this.targets,
        d: [
          this.defaultPath,
          'm479.26 557.95c-28.384-4.9278-59.225-12.92-79.873-33.009-16.528-16.081-35.961-39.909-28.165-63.189 7.7406-23.114 20.135-30.334 37.189-26.936 17.933 3.5728 29.469 2.8935 51.534 6.5091 16.327 2.6754 77.527 2.4356 97.698-0.82239 18.364-2.9662 37.681-4.1413 55.393-5.3859 14.648-1.0294 32.355 18.768 32.884 31.192 1.3491 31.677-22.376 51.186-35.492 62.319-13.835 11.744-30.682 20.559-48.192 25.329-26.717 7.2793-55.694 8.7304-82.976 3.9937z'
        ],
        direction: 'alternate',
        easing: 'steps(2)',
        duration: 200,
        loop: true
      })
      console.log('speech animation started')
    },
    async stop () {
      if (!this.anime) return
      this.anime.loopComplete = async (anim) => {
        // direction: alterlateのため、2回目のループまで待つ
        if (!anim.reversed) return
        anim.pause()
        await sleep(100)
        $(this.targets).attr('d', this.defaultPath)
        console.log('speech animation stopped')
      }
    }
  }
}
