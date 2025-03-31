const emojis = {
    none:{
        start(){},
        stop(){}
    },
    listen:{
        anime: null,
        start(){
            $('.listen').show()
            if (this.anime) {
                this.anime.restart()
            } else {
                this.anime = anime.timeline({
                    easing: 'linear',
                    loop: true,
                    direction: 'alternate'
                }).add({
                    targets: '.listen',
                    opacity: [0.4, 1],
                    duration: 1000
                })
            }
        },
        stop(){
            this.anime.pause()
            $('.listen').hide()
        }
    },
    notice:{
        anime: null,
        start(){
            $('.notice').show()
            if (this.anime) {
                this.anime.restart()
            } else {
                this.anime = anime.timeline({
                    easing: 'easeOutBack',
                    loop: true,
                }).add({
                    targets: '.notice',
                    translateY: [0, -50],
                    opacity: [0,1],
                    duration: 500
                }).add({
                    targets: '.notice',
                    opacity: [1,0],
                    duration: 800
                },'+=2000')
            }
        },
        stop(){
            this.anime.pause()
            $('.notice').hide()
        }
    },
    lost:{
        anime: null,
        start(){
            $('.lost').show()
            if (this.anime) {
                this.anime.restart()
            } else {
                this.anime = anime.timeline({
                    easing: 'easeOutQuart',
                    loop: true,
                }).add({
                    targets: '.lost',
                    translateY: [10, -50],
                    opacity: [0,1],
                    duration: 1300
                },0).add({
                    targets: '.lost',
                    opacity: [1,0],
                    duration: 800
                },'+=1000')
            }
        },
        stop(){
            this.anime.pause()
            $('.lost').hide()
        }
    },
    think: {
        anime: null,
        start(){
            $('.think').show()
            if (this.anime) {
                this.anime.restart()
            } else {
                this.anime = anime.timeline({
                    easing: 'linear',
                    loop: true
                }).add({
                    targets: '.think path',
                    opacity: [0, 1],
                    duration: 1000,
                    delay: anime.stagger(1000/3)
                }).add({
                    targets: '.think path',
                    opacity: [1, 0],
                    duration: 1000,
                    delay: anime.stagger(1000/3)
                }, '+=1000')
            }
        },
        stop(){
            this.anime.pause()
            $('.think').hide()
        }
    }
}
