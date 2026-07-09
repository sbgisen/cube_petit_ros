const curious = {
  timer: null,
  start () {
    emoteFadeIn('.curious')
    this.look()
  },
  look () {
    // きょろきょろと色々な方向を見る(8方向からランダム)
    const r = anime.random(60, 100) / 100
    const euler = anime.random(0, 7) / 8 * Math.PI * 2
    lookAt(r, euler).then(() => {
      this.timer = setTimeout(() => {
        // たまに中央に戻ってひと呼吸
        if (anime.random(1, 100) <= 30) {
          lookAt(0, 0).then(() => {
            this.timer = setTimeout(() => { this.look() }, anime.random(500, 1200))
          })
        } else {
          this.look()
        }
      }, anime.random(400, 1400))
    })
  },
  stop () {
    clearTimeout(this.timer)
    resetFace()
    emoteFadeOut('.curious')
  },
  speech: makeSpeech(
    '.curious .mouth path',
    'm479.26 557.95c-28.384-4.9278-59.225-12.92-79.873-33.009-16.528-16.081-35.961-39.909-28.165-63.189 7.7406-23.114 20.135-30.334 37.189-26.936 17.933 3.5728 29.469 2.8935 51.534 6.5091 16.327 2.6754 77.527 2.4356 97.698-0.82239 18.364-2.9662 37.681-4.1413 55.393-5.3859 14.648-1.0294 32.355 18.768 32.884 31.192 1.3491 31.677-22.376 51.186-35.492 62.319-13.835 11.744-30.682 20.559-48.192 25.329-26.717 7.2793-55.694 8.7304-82.976 3.9937z'
  )
}
