/**
 * @license
 *
 * Copyright 2019 LuxAI S.A.
 */

/**
 * @fileoverview Main QT robot code
 * @author ali.paikan@luxai.com (Ali Paikan)
 */
'use strict';

// when life is settled, load up the fun stuff
document.addEventListener('DOMContentLoaded', function () {
    var url = prompt("Please enter QTrobot rosbridge url:", "ws://192.168.100.2:9091");
    url = (url == null) ? 'ws://192.168.100.2:9091' : url;

    var application = new Vue({
        el: '#app',
        vuetify: new Vuetify({
            theme: { dark: false }
        }),
        data: {
            qtrobot : null,
            info_message: null,
            error_message: null,
            tts_message: ''
        },
        created() {
            var self = this;            
            self.info_message = "Connecting to Qtrobot (please wait...)";             
            // console.log("connecting  to QTrobot (please wait...)");
            self.qtrobot = new QTrobot({
                url : url,
                connection: function(){        
                    self.error_message = null;
                    self.info_message = null;
                },
                error: function(error){
                    console.log(error);
                },
                close: function(){
                    self.error_message = "Disconnected!";
                    self.info_message = null;                    
                }
            }); //end of qtrobot


        }, // end of create
        destroyed() {
          // 
        },
        mounted(){            
            // 
        },
        methods: {
            say_text() {
                console.log(this.tts_message);
                this.qtrobot.say_text(this.tts_message);
            }
            // 
        }, // end of method
        computed: {
            // 
        } // end of computhed
    }); // end of Vue

});
