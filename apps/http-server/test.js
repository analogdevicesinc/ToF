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


	var mirror_name = "";
	if (params.mirror)
		mirror_name = params.mirror;
	// console.log(mirror_name);

	var canvas;
	var context;
	var imgData;
	var data;

	var firstFrameDrawn = false;

	function updateFrame(binaryArray) {
		data = imgData.data;
		var j = 0;

		if (formatType == "ir") {
			for (var i = 0; i < data.length; i += 4) {
				data[i] = binaryArray[j];
				data[i + 1] = binaryArray[j];
				data[i + 2] = binaryArray[j];
				data[i + 3] = 255;
				j++;
			}
		}
		else if (formatType == "depth") {
			for (var i = 0; i < data.length; i += 4) {
				data[i] = 255 - binaryArray[j];
				data[i + 1] = 0;
				data[i + 2] = binaryArray[j];
				data[i + 3] = 255;
				j++;
			}
		}
		else
		{
			console.log("Unknown format type!");
			isStreaming = false;
		}
		context.putImageData(imgData, 0, 0);
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
					if (isStreaming == true)
						socket_tof.send("requestFrame\n");
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
							canvas = document.getElementById("canvas");
							canvas.width = formats[0];
							canvas.height = formats[1];

							// Update variables for frame drawing
							context = canvas.getContext('2d');
							imgData = context.getImageData(0, 0, canvas.width, canvas.height);
							data = imgData.data;
						}
						else {
							console.log("Bad size format from server");
						}

					}
					else if (msg.data.includes("ready")) {
						// Enabling start/stop buttons 
						enableStartStop();
					}
					else {
						document.getElementById("number").textContent = msg.data + "\n";
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
		}
		else {
			console.log("Could not sned format data!");
		}
	}

	// Start streaming
	function startStreaming() {
		if (socketConnected == true && camera != "" && frameType != "" && formatType != "") {
			// Start streaming
			isStreaming = true;
			console.log("Starting streaming");
			socket_tof.send("requestFrame\n");
		}
		else {
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
		document.getElementById("offset").onclick = reset;
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
