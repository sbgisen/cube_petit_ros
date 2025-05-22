/**
 * 顔をウィンドウの横サイズに合わせ
 * 垂直中央に配置する
 */
const el = $('.container')
const doc = document.documentElement.getBoundingClientRect()
const s = doc.width / el.width()
const boxLeft = Math.round(el.width() * (s - 1) / 2)
const boxTop = Math.round(
  el.height() * (s - 1) / 2 +
  (doc.height - el.height() * s) / 2 -
  50 // 画像が下寄りのため補正
)
el.css({
  transform: 'scale(' + s + ')',
  'margin-top': boxTop + 'px',
  'margin-left': boxLeft + 'px'
})
