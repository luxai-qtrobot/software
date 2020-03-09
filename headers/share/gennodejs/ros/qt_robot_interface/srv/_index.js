
"use strict";

let emotion_show = require('./emotion_show.js')
let gesture_play = require('./gesture_play.js')
let speech_config = require('./speech_config.js')
let speech_say = require('./speech_say.js')
let behavior_talk_audio = require('./behavior_talk_audio.js')
let audio_play = require('./audio_play.js')
let setting_setVolume = require('./setting_setVolume.js')
let behavior_talk_text = require('./behavior_talk_text.js')

module.exports = {
  emotion_show: emotion_show,
  gesture_play: gesture_play,
  speech_config: speech_config,
  speech_say: speech_say,
  behavior_talk_audio: behavior_talk_audio,
  audio_play: audio_play,
  setting_setVolume: setting_setVolume,
  behavior_talk_text: behavior_talk_text,
};
