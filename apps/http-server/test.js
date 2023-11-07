(function () {

        /**
         * Console logging for webapp messages
         */

        if (typeof console != "undefined")
                if (typeof console.log != 'undefined')
                        console.olog = console.log;
                else
                        console.olog = function () { };

        console.log = function (message) {
                console.olog(message);
                document.getElementById("tofConsole").innerHTML += Date.now() + ": " + message + "<br />";
        };
        console.error = console.debug = console.info = console.log


        /*
         * We display untrusted stuff in html context... reject anything
         * that has HTML stuff in it
         */

        function san(s) {
                if (s.search("<") !== -1)
                        return "invalid string";

                return s;
        }

        /* BrowserDetect came from http://www.quirksmode.org/js/detect.html */

        var BrowserDetect = {
                init: function () {
                        this.browser = this.searchString(this.dataBrowser) ||
                                "An unknown browser";
                        this.version = this.searchVersion(navigator.userAgent)
                                || this.searchVersion(navigator.appVersion)
                                || "an unknown version";
                        this.OS = this.searchString(this.dataOS) || "an unknown OS";
                },
                searchString: function (data) {
                        for (var i = 0; i < data.length; i++) {
                                var dataString = data[i].string;
                                var dataProp = data[i].prop;
                                this.versionSearchString = data[i].versionSearch || data[i].identity;
                                if (dataString) {
                                        if (dataString.indexOf(data[i].subString) !== -1)
                                                return data[i].identity;
                                }
                                else if (dataProp)
                                        return data[i].identity;
                        }
                },
                searchVersion: function (dataString) {
                        var index = dataString.indexOf(this.versionSearchString);
                        if (index === -1) return 0;
                        return parseFloat(dataString.substring(index +
                                this.versionSearchString.length + 1));
                },
                dataBrowser: [
                        {
                                string: navigator.userAgent,
                                subString: "Chrome",
                                identity: "Chrome"
                        },
                        {
                                string: navigator.userAgent,
                                subString: "OmniWeb",
                                versionSearch: "OmniWeb/",
                                identity: "OmniWeb"
                        },
                        {
                                string: navigator.vendor,
                                subString: "Apple",
                                identity: "Safari",
                                versionSearch: "Version"
                        },
                        {
                                prop: window.opera,
                                identity: "Opera",
                                versionSearch: "Version"
                        },
                        {
                                string: navigator.vendor,
                                subString: "iCab",
                                identity: "iCab"
                        },
                        {
                                string: navigator.vendor,
                                subString: "KDE",
                                identity: "Konqueror"
                        },
                        {
                                string: navigator.userAgent,
                                subString: "Firefox",
                                identity: "Firefox"
                        },
                        {
                                string: navigator.vendor,
                                subString: "Camino",
                                identity: "Camino"
                        },
                        {		// for newer Netscapes (6+)
                                string: navigator.userAgent,
                                subString: "Netscape",
                                identity: "Netscape"
                        },
                        {
                                string: navigator.userAgent,
                                subString: "MSIE",
                                identity: "Explorer",
                                versionSearch: "MSIE"
                        },
                        {
                                string: navigator.userAgent,
                                subString: "Gecko",
                                identity: "Mozilla",
                                versionSearch: "rv"
                        },
                        { 		// for older Netscapes (4-)
                                string: navigator.userAgent,
                                subString: "Mozilla",
                                identity: "Netscape",
                                versionSearch: "Mozilla"
                        }
                ],
                dataOS: [
                        {
                                string: navigator.platform,
                                subString: "Win",
                                identity: "Windows"
                        },
                        {
                                string: navigator.platform,
                                subString: "Mac",
                                identity: "Mac"
                        },
                        {
                                string: navigator.userAgent,
                                subString: "iPhone",
                                identity: "iPhone/iPod"
                        },
                        {
                                string: navigator.platform,
                                subString: "Linux",
                                identity: "Linux"
                        }
                ]

        };

        var pos = 0;

        function get_appropriate_ws_url(extra_url) {
                var pcol;
                var u = document.URL;

                /*
                 * We open the websocket encrypted if this page came on an
                 * https:// url itself, otherwise unencrypted
                 */
                if (u.substring(0, 5) === "https") {
                        pcol = "wss://";
                        u = u.substr(8);
                } else {
                        pcol = "ws://";
                        if (u.substring(0, 4) === "http")
                                u = u.substr(7);
                }
                u = u.split("/");
                /* + "/xxx" bit is for IE10 workaround */
                return pcol + u[0] + "/" + extra_url;
        }

        var params = {};

        if (location.search) {
                var parts = location.search.substring(1).split("&");

                for (var i = 0; i < parts.length; i++) {
                        var nv = parts[i].split("=");
                        if (!nv[0]) continue;
                        params[nv[0]] = nv[1] || true;
                }
        }

        var socket_tof;
        var socketConnected = false;
        var state = 0;
        var camera = "";
        var frameType = "";
        var formatType = "";
        var isPreparingFrame = false;
        var isStreaming = false;

        var colormap = [
                { red: 0, green: 0, blue: 255 },
                { red: 0, green: 2, blue: 255 },
                { red: 0, green: 5, blue: 255 },
                { red: 0, green: 8, blue: 255 },
                { red: 0, green: 10, blue: 255 },
                { red: 0, green: 12, blue: 255 },
                { red: 0, green: 15, blue: 255 },
                { red: 0, green: 18, blue: 255 },
                { red: 0, green: 20, blue: 255 },
                { red: 0, green: 22, blue: 255 },
                { red: 0, green: 25, blue: 255 },
                { red: 0, green: 27, blue: 255 },
                { red: 0, green: 30, blue: 255 },
                { red: 0, green: 32, blue: 255 },
                { red: 0, green: 35, blue: 255 },
                { red: 0, green: 38, blue: 255 },
                { red: 0, green: 40, blue: 255 },
                { red: 0, green: 42, blue: 255 },
                { red: 0, green: 45, blue: 255 },
                { red: 0, green: 48, blue: 255 },
                { red: 0, green: 50, blue: 255 },
                { red: 0, green: 52, blue: 255 },
                { red: 0, green: 55, blue: 255 },
                { red: 0, green: 57, blue: 255 },
                { red: 0, green: 60, blue: 255 },
                { red: 0, green: 62, blue: 255 },
                { red: 0, green: 65, blue: 255 },
                { red: 0, green: 68, blue: 255 },
                { red: 0, green: 70, blue: 255 },
                { red: 0, green: 72, blue: 255 },
                { red: 0, green: 75, blue: 255 },
                { red: 0, green: 78, blue: 255 },
                { red: 0, green: 80, blue: 255 },
                { red: 0, green: 82, blue: 255 },
                { red: 0, green: 85, blue: 255 },
                { red: 0, green: 88, blue: 255 },
                { red: 0, green: 90, blue: 255 },
                { red: 0, green: 92, blue: 255 },
                { red: 0, green: 95, blue: 255 },
                { red: 0, green: 98, blue: 255 },
                { red: 0, green: 100, blue: 255 },
                { red: 0, green: 102, blue: 255 },
                { red: 0, green: 105, blue: 255 },
                { red: 0, green: 108, blue: 255 },
                { red: 0, green: 110, blue: 255 },
                { red: 0, green: 112, blue: 255 },
                { red: 0, green: 115, blue: 255 },
                { red: 0, green: 117, blue: 255 },
                { red: 0, green: 120, blue: 255 },
                { red: 0, green: 122, blue: 255 },
                { red: 0, green: 125, blue: 255 },
                { red: 0, green: 128, blue: 255 },
                { red: 0, green: 130, blue: 255 },
                { red: 0, green: 132, blue: 255 },
                { red: 0, green: 135, blue: 255 },
                { red: 0, green: 138, blue: 255 },
                { red: 0, green: 140, blue: 255 },
                { red: 0, green: 142, blue: 255 },
                { red: 0, green: 145, blue: 255 },
                { red: 0, green: 148, blue: 255 },
                { red: 0, green: 150, blue: 255 },
                { red: 0, green: 152, blue: 255 },
                { red: 0, green: 155, blue: 255 },
                { red: 0, green: 158, blue: 255 },
                { red: 0, green: 160, blue: 255 },
                { red: 0, green: 162, blue: 255 },
                { red: 0, green: 165, blue: 255 },
                { red: 0, green: 168, blue: 255 },
                { red: 0, green: 170, blue: 255 },
                { red: 0, green: 172, blue: 255 },
                { red: 0, green: 175, blue: 255 },
                { red: 0, green: 178, blue: 255 },
                { red: 0, green: 180, blue: 255 },
                { red: 0, green: 182, blue: 255 },
                { red: 0, green: 185, blue: 255 },
                { red: 0, green: 188, blue: 255 },
                { red: 0, green: 190, blue: 255 },
                { red: 0, green: 192, blue: 255 },
                { red: 0, green: 195, blue: 255 },
                { red: 0, green: 198, blue: 255 },
                { red: 0, green: 200, blue: 255 },
                { red: 0, green: 202, blue: 255 },
                { red: 0, green: 205, blue: 255 },
                { red: 0, green: 208, blue: 255 },
                { red: 0, green: 210, blue: 255 },
                { red: 0, green: 212, blue: 255 },
                { red: 0, green: 215, blue: 255 },
                { red: 0, green: 218, blue: 255 },
                { red: 0, green: 220, blue: 255 },
                { red: 0, green: 223, blue: 255 },
                { red: 0, green: 225, blue: 255 },
                { red: 0, green: 228, blue: 255 },
                { red: 0, green: 230, blue: 255 },
                { red: 0, green: 232, blue: 255 },
                { red: 0, green: 235, blue: 255 },
                { red: 0, green: 238, blue: 255 },
                { red: 0, green: 240, blue: 255 },
                { red: 0, green: 243, blue: 255 },
                { red: 0, green: 245, blue: 255 },
                { red: 0, green: 248, blue: 255 },
                { red: 0, green: 250, blue: 255 },
                { red: 0, green: 252, blue: 255 },
                { red: 0, green: 253, blue: 252 },
                { red: 0, green: 254, blue: 248 },
                { red: 0, green: 254, blue: 244 },
                { red: 0, green: 255, blue: 240 },
                { red: 0, green: 255, blue: 235 },
                { red: 0, green: 255, blue: 230 },
                { red: 0, green: 255, blue: 225 },
                { red: 0, green: 255, blue: 220 },
                { red: 0, green: 255, blue: 215 },
                { red: 0, green: 255, blue: 210 },
                { red: 0, green: 255, blue: 205 },
                { red: 0, green: 255, blue: 200 },
                { red: 0, green: 255, blue: 195 },
                { red: 0, green: 255, blue: 190 },
                { red: 0, green: 255, blue: 185 },
                { red: 0, green: 255, blue: 180 },
                { red: 0, green: 255, blue: 175 },
                { red: 0, green: 255, blue: 170 },
                { red: 0, green: 255, blue: 165 },
                { red: 0, green: 255, blue: 160 },
                { red: 0, green: 255, blue: 155 },
                { red: 0, green: 255, blue: 150 },
                { red: 0, green: 255, blue: 145 },
                { red: 0, green: 255, blue: 140 },
                { red: 0, green: 255, blue: 135 },
                { red: 0, green: 255, blue: 130 },
                { red: 0, green: 255, blue: 125 },
                { red: 0, green: 255, blue: 120 },
                { red: 0, green: 255, blue: 115 },
                { red: 0, green: 255, blue: 110 },
                { red: 0, green: 255, blue: 105 },
                { red: 0, green: 255, blue: 100 },
                { red: 0, green: 255, blue: 95 },
                { red: 0, green: 255, blue: 90 },
                { red: 0, green: 255, blue: 85 },
                { red: 0, green: 255, blue: 80 },
                { red: 0, green: 255, blue: 75 },
                { red: 0, green: 255, blue: 70 },
                { red: 0, green: 255, blue: 65 },
                { red: 0, green: 255, blue: 60 },
                { red: 0, green: 255, blue: 55 },
                { red: 0, green: 255, blue: 50 },
                { red: 0, green: 255, blue: 45 },
                { red: 0, green: 255, blue: 40 },
                { red: 0, green: 255, blue: 35 },
                { red: 0, green: 255, blue: 30 },
                { red: 0, green: 255, blue: 25 },
                { red: 0, green: 255, blue: 20 },
                { red: 0, green: 255, blue: 15 },
                { red: 1, green: 254, blue: 11 },
                { red: 2, green: 253, blue: 7 },
                { red: 3, green: 252, blue: 3 },
                { red: 5, green: 250, blue: 0 },
                { red: 10, green: 245, blue: 0 },
                { red: 15, green: 240, blue: 0 },
                { red: 20, green: 235, blue: 0 },
                { red: 25, green: 230, blue: 0 },
                { red: 30, green: 225, blue: 0 },
                { red: 35, green: 220, blue: 0 },
                { red: 40, green: 215, blue: 0 },
                { red: 45, green: 210, blue: 0 },
                { red: 50, green: 205, blue: 0 },
                { red: 55, green: 200, blue: 0 },
                { red: 60, green: 195, blue: 0 },
                { red: 65, green: 190, blue: 0 },
                { red: 70, green: 185, blue: 0 },
                { red: 75, green: 180, blue: 0 },
                { red: 80, green: 175, blue: 0 },
                { red: 85, green: 170, blue: 0 },
                { red: 90, green: 165, blue: 0 },
                { red: 95, green: 160, blue: 0 },
                { red: 100, green: 155, blue: 0 },
                { red: 105, green: 150, blue: 0 },
                { red: 110, green: 145, blue: 0 },
                { red: 115, green: 140, blue: 0 },
                { red: 120, green: 135, blue: 0 },
                { red: 125, green: 130, blue: 0 },
                { red: 130, green: 125, blue: 0 },
                { red: 135, green: 120, blue: 0 },
                { red: 140, green: 115, blue: 0 },
                { red: 145, green: 110, blue: 0 },
                { red: 150, green: 105, blue: 0 },
                { red: 155, green: 100, blue: 0 },
                { red: 160, green: 95, blue: 0 },
                { red: 165, green: 90, blue: 0 },
                { red: 170, green: 85, blue: 0 },
                { red: 175, green: 80, blue: 0 },
                { red: 180, green: 75, blue: 0 },
                { red: 185, green: 70, blue: 0 },
                { red: 190, green: 65, blue: 0 },
                { red: 195, green: 60, blue: 0 },
                { red: 200, green: 55, blue: 0 },
                { red: 205, green: 50, blue: 0 },
                { red: 210, green: 45, blue: 0 },
                { red: 215, green: 40, blue: 0 },
                { red: 220, green: 35, blue: 0 },
                { red: 225, green: 30, blue: 0 },
                { red: 230, green: 25, blue: 0 },
                { red: 235, green: 20, blue: 0 },
                { red: 240, green: 15, blue: 0 },
                { red: 245, green: 10, blue: 0 },
                { red: 248, green: 7, blue: 1 },
                { red: 250, green: 5, blue: 3 },
                { red: 252, green: 3, blue: 5 },
                { red: 254, green: 1, blue: 7 },
                { red: 255, green: 0, blue: 10 },
                { red: 255, green: 0, blue: 13 },
                { red: 255, green: 0, blue: 17 },
                { red: 255, green: 0, blue: 20 },
                { red: 255, green: 0, blue: 23 },
                { red: 255, green: 0, blue: 27 },
                { red: 255, green: 0, blue: 30 },
                { red: 255, green: 0, blue: 33 },
                { red: 255, green: 0, blue: 37 },
                { red: 255, green: 0, blue: 40 },
                { red: 255, green: 0, blue: 43 },
                { red: 255, green: 0, blue: 47 },
                { red: 255, green: 0, blue: 50 },
                { red: 255, green: 0, blue: 53 },
                { red: 255, green: 0, blue: 57 },
                { red: 255, green: 0, blue: 60 },
                { red: 255, green: 0, blue: 63 },
                { red: 255, green: 0, blue: 67 },
                { red: 255, green: 0, blue: 70 },
                { red: 255, green: 0, blue: 73 },
                { red: 255, green: 0, blue: 77 },
                { red: 255, green: 0, blue: 80 },
                { red: 255, green: 0, blue: 83 },
                { red: 255, green: 0, blue: 87 },
                { red: 255, green: 0, blue: 90 },
                { red: 255, green: 0, blue: 93 },
                { red: 255, green: 0, blue: 97 },
                { red: 255, green: 0, blue: 100 },
                { red: 255, green: 0, blue: 103 },
                { red: 255, green: 0, blue: 107 },
                { red: 255, green: 0, blue: 110 },
                { red: 255, green: 0, blue: 113 },
                { red: 255, green: 0, blue: 117 },
                { red: 255, green: 0, blue: 120 },
                { red: 255, green: 0, blue: 123 },
                { red: 255, green: 0, blue: 127 },
                { red: 255, green: 0, blue: 130 },
                { red: 255, green: 0, blue: 133 },
                { red: 255, green: 0, blue: 137 },
                { red: 255, green: 0, blue: 140 },
                { red: 255, green: 0, blue: 143 },
                { red: 255, green: 0, blue: 147 },
                { red: 255, green: 0, blue: 150 },
                { red: 255, green: 0, blue: 153 },
                { red: 255, green: 0, blue: 157 },
                { red: 255, green: 0, blue: 160 },
                { red: 255, green: 0, blue: 163 },
                { red: 255, green: 0, blue: 167 },
                { red: 255, green: 0, blue: 17 }
        ];

        // To Remove
        var prevTime = Date.now();
        var curTime = Date.now();

        var mirror_name = "";
        if (params.mirror)
                mirror_name = params.mirror;
        // console.log(mirror_name);

        // Canvass data
        var canvasWidth;
        var canvasHeight;

        // Canvas 1
        var canvas1;
        var context1;
        var imgData1;
        var data1;

        // Canvas 2
        var canvas2;
        var context2;
        var imgData2;
        var data2;

        var firstFrameDrawn = false;

        function updateFrame(binaryArray) {
                var j = 0;
                if (formatType == "depth") {
                        data1 = imgData1.data;

                        for (var i = 0; i < data1.length; i += 4) {
                                // Get data from lookup table
                                data1[i] = colormap[255 - binaryArray[j]].red;
                                data1[i + 1] = colormap[255 - binaryArray[j]].green;
                                data1[i + 2] = colormap[255 - binaryArray[j]].blue;
                                data1[i + 3] = 200;
                                j++;
                        }
                        context1.putImageData(imgData1, 0, 0);

                }
                else if (formatType == "ir") {
                        data1 = imgData1.data;

                        for (var i = 0; i < data1.length; i += 4) {
                                data1[i] = binaryArray[j];
                                data1[i + 1] = binaryArray[j];
                                data1[i + 2] = binaryArray[j];
                                data1[i + 3] = 200;
                                j++;
                        }
                        context1.putImageData(imgData1, 0, 0);

                }
                else if (formatType == "depth+ir") {
                        data1 = imgData1.data;
                        data2 = imgData2.data;
                        for (var i = 0; i < data1.length; i += 4) {
                                // Get data from lookup table
                                data1[i] = colormap[255 - binaryArray[j]].red;
                                data1[i + 1] = colormap[255 - binaryArray[j]].green;
                                data1[i + 2] = colormap[255 - binaryArray[j]].blue;
                                data1[i + 3] = 200;
                                j++;
                        }
                        for (var i = 0; i < data2.length; i += 4) {
                                data2[i] = binaryArray[j];
                                data2[i + 1] = binaryArray[j];
                                data2[i + 2] = binaryArray[j];
                                data2[i + 3] = 200;
                                j++;
                        }
                        context1.putImageData(imgData1, 0, 0);
                        context2.putImageData(imgData2, 0, 0);
                }
                else {
                        console.log("Unknown format type!");
                        isStreaming = false;
                }

                isPreparingFrame = false;
        }


        const blobToBinary = async (blob) => {
                isPreparingFrame = true;
                const buffer = await blob.arrayBuffer();
                const frame = new Uint8Array(buffer);
                updateFrame(frame);
        };

        function ws_open_tof() {
                socket_tof = new_ws(get_appropriate_ws_url(""), "dumb-increment-protocol");

                try {
                        socket_tof.onopen = function () {
                                // lws_gray_out(false);
                                document.getElementById("wsdi_statustd").style.backgroundColor =
                                        "#40ff40";
                                document.getElementById("wsdi_status").innerHTML =
                                        " <b>websocket connection opened</b><br>" +
                                        san(socket_tof.extensions);
                                console.log("Network connection opened");
                                socketConnected = true;
                                state = 1;
                        };

                        socket_tof.onmessage = async function got_packet(msgGot) {
                                let msg = msgGot;

                                if (msg.data instanceof Blob) {
                                        if (isPreparingFrame == false)
                                                blobToBinary(msg.data);
                                        if (isStreaming == true) {
                                                socket_tof.send("requestFrame\n");
                                                curTime = Date.now();
                                                fps = Math.round((1000.0) / (curTime - prevTime));
                                                prevTime = curTime;
                                                document.getElementById("fps").textContent = "Camera fps: " + fps + "\n";
                                        }
                                }
                                else {
                                        if (msg.data.includes("ft:")) {
                                                // Delete previous frame types:
                                                var i, L = document.getElementById("frameTypeSelect").options.length - 1;
                                                for (i = L; i >= 0; i--) {
                                                        document.getElementById("frameTypeSelect").remove(i);
                                                }

                                                var message = msg.data.substring(msg.data.indexOf(":") + 1);
                                                const frameTypes = message.split(',');
                                                var i = 0;
                                                while (i < frameTypes.length) {
                                                        var newOption = document.createElement("option");
                                                        newOption.value = frameTypes[i];
                                                        newOption.text = frameTypes[i];
                                                        document.getElementById("frameTypeSelect").add(newOption, document.getElementById("frameTypeSelect")[i]);
                                                        i++;
                                                }
                                                enableFrameType();

                                        }
                                        else if (msg.data.includes("format:")) {
                                                // Delete previous formats:
                                                var i, L = document.getElementById("formatSelect").options.length - 1;
                                                for (i = L; i >= 0; i--) {
                                                        document.getElementById("formatSelect").remove(i);
                                                }

                                                // Add new formats
                                                var message = msg.data.substring(msg.data.indexOf(":") + 1);
                                                const formats = message.split(',');
                                                var i = 0;
                                                while (i < formats.length) {
                                                        var newOption = document.createElement("option");
                                                        newOption.value = formats[i];
                                                        newOption.text = formats[i];
                                                        document.getElementById("formatSelect").add(newOption, document.getElementById("formatSelect")[i]);
                                                        i++;
                                                }
                                                enableFormat();

                                        }
                                        else if (msg.data.includes("size:")) {
                                                var message = msg.data.substring(msg.data.indexOf(":") + 1);
                                                const formats = message.split(',');
                                                if (formats.length == 2) {
                                                        canvasWidth = formats[0];
                                                        canvasHeight = formats[1];
                                                }
                                                else {
                                                        console.log("Bad size format from server");
                                                }

                                        }
                                        else if (msg.data.includes("ready")) {
                                                // Enabling start/stop buttons 
                                                enableStartStop();
                                        }
                                        else if (msg.data.includes("error_log")) {
                                                // Logging error on host side
                                                var message = msg.data.substring(msg.data.indexOf(":") + 1);
                                                console.log(message);
                                        }
                                        else {
                                                chars = "|/-\\";
                                                document.getElementById("number").textContent = chars.charAt(msg.data % 4) + "\n";
                                        }
                                }

                        };

                        socket_tof.onclose = function () {
                                // lws_gray_out(true,{"zindex":"50"});
                                document.getElementById("wsdi_statustd").style.backgroundColor =
                                        "#ff4040";
                                document.getElementById("wsdi_status").textContent =
                                        " websocket connection CLOSED ";
                                console.log("Network connection closed");

                                disableFrameType();

                                socketConnected = false;
                                state = 0;
                        };

                } catch (exception) {
                        alert("<p>Error" + exception);
                }
        }

        var socket_status, jso, s;
        function reset() {
                socket_tof.send("reset\n");
        }

        //Open camera
        function openCamera() {
                if (socketConnected == true) {
                        if (document.getElementById('connectCamera').value == "Open") {
                                camera = document.getElementById('cameraSelect').value;
                                console.log("Opening camera: " + camera);
                                socket_tof.send("open:" + camera + "\n");
                                document.getElementById('cameraSelect').disabled = true;
                                document.getElementById('connectCamera').value = "Close"
                                state = 2;
                                enableFrameType();
                        }
                        else {
                                console.log("Closing camera: " + camera);
                                socket_tof.send("close\n");
                                camera = "";
                                document.getElementById('cameraSelect').disabled = false;
                                document.getElementById('connectCamera').value = "Open"
                                state = 1;
                                disableFrameType();

                        }
                }
                else {
                        console.log("No socket connected to server!");
                }
        }

        // Set Frame Type
        function setFrameType() {
                disableFormat();
                if (socketConnected == true && camera != "") {
                        // Sending frame type
                        frameType = document.getElementById("frameTypeSelect").value;
                        console.log("Setting frame type: " + frameType);
                        socket_tof.send("setft:" + frameType + "\n");
                }
                else {
                        console.log("Network/camera error!");
                }
        }

        // Set Format 
        function setFormat() {
                if (socketConnected == true && camera != "" && frameType != "") {
                        // Sending frame type
                        formatType = document.getElementById("formatSelect").value;
                        console.log("Setting formt: " + formatType);
                        socket_tof.send("setFormat:" + formatType + "\n");

                        // Canvas 1
                        canvas1 = document.getElementById("canvas1");
                        canvas1.width = canvasWidth;
                        canvas1.height = canvasHeight;
                        context1 = canvas1.getContext('2d');
                        imgData1 = context1.getImageData(0, 0, canvas1.width, canvas1.height);
                        data1 = imgData1.data;

                        // Canvas 2
                        canvas2 = document.getElementById("canvas2");
                        if (formatType == "depth+ir") {
                                canvas2.width = canvasWidth;
                                canvas2.height = canvasHeight;
                                context2 = canvas2.getContext('2d');
                                imgData2 = context2.getImageData(0, 0, canvas2.width, canvas2.height);
                                data2 = imgData2.data;
                        }
                        else {
                                canvas2.width = 0;
                                canvas2.height = 0;
                        }

                }
                else {
                        console.log("Could not send format data!");
                }
        }

        // Start streaming
        function startStreaming() {
                if (socketConnected == true && camera != "" && frameType != "" && formatType != "") {
                        // Start streaming
                        isStreaming = true;
                        console.log("Starting streaming");
                        socket_tof.send("requestFrame\n");
                        document.getElementById("fps").style.display = "visible";
                }
                else {
                        document.getElementById("fps").style.display = "hidden";
                        console.log("Could not start streaming");
                }
        }

        // Stop streaming
        function stopStreaming() {
                if (socketConnected == true && camera != "" && frameType != "" && formatType != "") {
                        // Stop streaming
                        isStreaming = false;
                        console.log("Stop streaming");
                        socket_tof.send("stop\n");
                        document.getElementById("fps").style.display = "hidden";

                }
                else {
                        console.log("Could not stop streaming");
                }
        }

        // Enable/disable controls

        function enableFrameType() {
                document.getElementById("frameTypeSelect").disabled = false;
                document.getElementById("setFrameType").disabled = false;
        }

        function enableFormat() {
                document.getElementById("formatSelect").disabled = false;
                document.getElementById("setFormat").disabled = false;
        }

        function enableStartStop() {
                document.getElementById("start").disabled = false;
                document.getElementById("stop").disabled = false;
        }

        function disableFrameType() {
                disableFormat();

                // Delete previous frame types:
                var i, L = document.getElementById("frameTypeSelect").options.length - 1;
                for (i = L; i >= 0; i--) {
                        document.getElementById("frameTypeSelect").remove(i);
                }

                document.getElementById("frameTypeSelect").disabled = true;
                document.getElementById("setFrameType").disabled = true;
        }

        function disableFormat() {
                disableStartStop();

                // Delete previous formats:
                var i, L = document.getElementById("formatSelect").options.length - 1;
                for (i = L; i >= 0; i--) {
                        document.getElementById("formatSelect").remove(i);
                }

                document.getElementById("formatSelect").disabled = true;
                document.getElementById("setFormat").disabled = true;
        }

        function disableStartStop() {
                document.getElementById("start").disabled = true;
                document.getElementById("stop").disabled = true;
        }

        /* stuff that has to be delayed until all the page assets are loaded */

        window.addEventListener("load", function () {

                // lws_gray_out(true,{"zindex":"499"});
                // document.getElementById("offset").onclick = reset;
                document.getElementById("connectCamera").onclick = openCamera;
                document.getElementById("setFrameType").onclick = setFrameType;
                document.getElementById("setFormat").onclick = setFormat;
                document.getElementById("start").onclick = startStreaming;
                document.getElementById("stop").onclick = stopStreaming;

                var transport_protocol = "";

                if (performance && performance.timing.nextHopProtocol) {
                        transport_protocol = performance.timing.nextHopProtocol;
                } else if (window.chrome && window.chrome.loadTimes) {
                        transport_protocol = window.chrome.loadTimes().connectionInfo;
                } else {

                        var p = performance.getEntriesByType("resource");
                        for (var i = 0; i < p.length; i++) {
                                var value = "nextHopProtocol" in p[i];
                                if (value)
                                        transport_protocol = p[i].nextHopProtocol;
                        }
                }

                console.log("Transport protocol: " + transport_protocol);

                if (transport_protocol === "h2")
                        document.getElementById("transport").innerHTML =
                                "<img src=\"./http2.png\">";

                BrowserDetect.init();

                document.getElementById("brow").textContent = " " +
                        BrowserDetect.browser + " " + BrowserDetect.version + " " +
                        BrowserDetect.OS + " ";

                document.getElementById("number").textContent = get_appropriate_ws_url(mirror_name);

                /* create the ws connections back to the server */

                ws_open_tof();

        }, false);

}());
