// ***************************************************** System Tab ********************************************************************/

// ***************************************************** Modify Permission *************************************************************/

let isEventSourceClosed = false;

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function showOverlay(message = "Processing, please wait...") {
  const overlay = document.getElementById("overlay");
  const overlayText = document.getElementById("overlay-text");
  overlayText.textContent = message;
  overlay.style.display = "flex";
}

function hideOverlay() {
  document.getElementById("overlay").style.display = "none";
}

async function setServerTime() {
  const browserTime = new Date();
  const timeZoneName = Intl.DateTimeFormat().resolvedOptions().timeZone; // Full timezone name
  const timeZoneAbbr = new Date()
    .toLocaleTimeString("en-us", { timeZoneName: "short" })
    .split(" ")[2]; // Short name like EST/EDT

  try {
    const response = await fetch("/set-server-time", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        browser_time: browserTime.toLocaleString("en-US", {
          timeZone: "Asia/Kolkata",
        }),
        time_zone_name: timeZoneName,
      }),
    });

    console.log(`TimeZone Name: ${timeZoneName}`);
    console.log(`TimeZone Abbreviation: ${timeZoneAbbr}`);

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.error || "Failed to set server time");
    }

    const result = await response.json();
    alert(result.message);
    updateSystemTab();
  } catch (error) {
    console.error("Error setting server time:", error);
    alert("Error setting server time: " + error.message);
  }
}

function showCountdownOverlay(seconds, action) {
  const overlayContent = document.querySelector(".overlay-content");
  document.getElementById("overlay").style.display = "flex";

  function updateCountdown() {
    if (seconds > 0) {
      overlayContent.innerHTML = `<div class="spinner"></div><p>${action} in ${seconds} seconds...</p>`;
      seconds--;
    } else {
      clearInterval(countdownInterval);
      overlayContent.innerHTML = `<div class="spinner"></div><p>System is ${action.toLowerCase()}...</p>`;
      if (action === "Rebooting") {
        initiateReboot();
      } else if (action === "Powering down") {
        initiatePowerDown();
      }
    }
  }

  updateCountdown();
  const countdownInterval = setInterval(updateCountdown, 1000);
}

function initiateReboot() {
  fetch("/reboot-system", { method: "POST" })
    .then((response) => {
      if (!response.ok) {
        console.error("Failed to reboot the system.");
      }
    })
    .catch((error) => {
      console.error("Error while attempting to reboot:", error);
    });

  document.querySelector(
    ".overlay-content"
  ).innerHTML = `<div class="spinner"></div><p>System is rebooting. Reconnecting soon...</p>`;
  pollServerReconnect();
}

function initiatePowerDown() {
  fetch("/power-down-system", { method: "POST" })
    .then((response) => {
      if (!response.ok) {
        console.error("Failed to power down the system.");
      }
    })
    .catch((error) => {
      console.error("Error while attempting to power down:", error);
    });

  document.querySelector(
    ".overlay-content"
  ).innerHTML = `<div class="spinner"></div><p>System is powering down...</p>`;
}

function pollServerReconnect() {
  const reconnectInterval = setInterval(async () => {
    try {
      const response = await fetch("/");
      if (response.ok) {
        hideOverlay();
        clearInterval(reconnectInterval);
      }
    } catch (error) {
      console.log("Server still rebooting, retrying...");
    }
  }, 5000);
}

function startRebootCountdown() {
  showCountdownOverlay(5, "Rebooting");
}

function startPowerDownCountdown() {
  showCountdownOverlay(5, "Powering down");
}

async function showPermissionOverlay() {
  const permissionOverlay = document.getElementById("permission-overlay");
  const permissionOverlayContent = document.getElementById(
    "permission-overlay-content"
  );

  try {
    const response = await fetch("/Change-Permission", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ value: "view" }),
    });

    if (!response.ok) {
      console.error("Failed to execute the script");
      return;
    }

    const newEvent = await fetch("/Change-Permission-Events");

    if (!newEvent.ok) {
      throw new Error("Failed to execute script");
    }

    const data = await newEvent.json();

    permissionOverlayContent.innerHTML = data.output;
    permissionOverlay.setAttribute("open", "true");

    // Close the overlay when clicking outside of it
    window.onclick = function (event) {
      if (event.target == permissionOverlay) {
        permissionOverlay.setAttribute("open", "false");
      }
    };
  } catch (error) {
    console.error("Error in Viewing Permission:", error);
  }
}

async function modifyPermission() {
  try {
    const response = await fetch("/Change-Permission", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ value: "modify" }),
    });

    if (!response.ok) {
      console.error("Failed to execute the script");
      return;
    }

    const newEvent = await fetch("/Change-Permission-Events");

    if (!newEvent.ok) {
      throw new Error("Failed to execute script");
    }

    const data = await newEvent.json();

    const userConfirmed = confirm(
      "Do you agree to " +
        data.output +
        "? If you agree, the system will restart to apply these changes."
    );
    if (userConfirmed) {
      startRebootCountdown();
    } else {
      const retreat_back = await fetch("/Change-Permission-Events");

      if (!retreat_back.ok) {
        throw new Error("Failed to execute script");
      }
    }
  } catch (error) {
    console.error("Error in Viewing Permission:", error);
  }
}

// ************************* Update display time ******************************* //

document.addEventListener("DOMContentLoaded", function () {
  updateDisplayedTimes();
});

async function updateSystemTab() {
  try {
    const response = await fetch("/server-time");
    const data = await response.json();
    const serverTimeUtc = data.server_time_utc;

    // Convert server time to browser's local timezone
    const serverTime = new Date(serverTimeUtc + " UTC");
    const browserTimeZone = Intl.DateTimeFormat().resolvedOptions().timeZone;
    const serverTimeInBrowserTZ = serverTime.toLocaleString("en-US", {
      timeZone: browserTimeZone,
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: true,
      timeZoneName: "short",
    });

    // Split the date and time
    const [date, time] = serverTimeInBrowserTZ.split(", ");

    // Update the table cells
    document.getElementById("evalDate").textContent = date;
    document.getElementById("evalTime").textContent = time;
  } catch (error) {
    console.error("Error fetching server time:", error);
    document.getElementById("evalDate").textContent = "Error";
    document.getElementById("evalTime").textContent = "Error";
  }

  const browserTime = new Date().toLocaleString("en-US", {
    timeZoneName: "short",
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: true,
  });
  // Split the date and time
  const [date, time] = browserTime.split(", ");

  // Update the table cells
  document.getElementById("browserDate").textContent = date;
  document.getElementById("browserTime").textContent = time;
}

function updateDisplayedTimes() {
  updateSystemTab();
}

// ******************************************************************* Read NVM/CCB **********************************************************/
const terminalOutput = document.getElementById("terminal_output");
async function NVMRead() {
  const modal = document.getElementById("read_tools_overlay_nvm");

  // Open the modal
  modal.setAttribute("open", "true");

  // Wait for the modal to close
  await new Promise((resolve) => {
    const handler = () => {
      modal.removeEventListener("modalCloseRequest", handler);
      resolve();
    };
    modal.addEventListener("modalCloseRequest", handler);
  });

  document
    .getElementById("read_tools_overlay_nvm")
    .setAttribute("open", "false");
}

function closePopup(id) {
  document.getElementById(id).setAttribute("open", "false");
}

function showPopUp_NVM(message) {
  const packetRegex = /Packet number : (\d+) \/ (\d+)/;

  // Split the current terminal output into lines
  const outputLines = terminalOutput.textContent.split("\n");
  let updated = false; // Flag to track whether a line was updated

  // Iterate through lines and update matching ones
  for (let i = 0; i < outputLines.length; i++) {
    if (packetRegex.test(outputLines[i]) && packetRegex.test(message)) {
      // Update Packet number line
      const match = message.match(packetRegex);
      const currentPacket = match[1];
      const totalPackets = match[2];
      outputLines[i] = `Packet number : ${currentPacket} / ${totalPackets}`;
      updated = true;
    }
  }

  // If no existing line was updated, append the new message
  if (!updated) {
    outputLines.push(message);
  }

  // Join the lines back and update the terminal output
  terminalOutput.textContent = outputLines.join("\n");
}

async function runNVMScript() {
  const fileName = document.getElementById("NVM_file-name").value;
  document
    .getElementById("read_tools_overlay_nvm")
    .setAttribute("open", "false");
  const path = "Tools/host_boot_tools/NVM_Utils";
  const script = "./nvm-read.sh";

  const response = await fetch("/run_shell_script", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ version: fileName, script: script }),
  });

  if (!response.ok) {
    console.error("Error starting shell script:", response.statusText);
    return;
  }
  const eventSource = new EventSource("/run-script");
  isEventSourceClosed = false;

  terminalOutput.textContent = ""; // Clear previous output
  document.getElementById("terminal_modal").setAttribute("open", "true");
  terminalOutput.textContent = "Reading NVM ...\n\n";

  eventSource.onmessage = function (event) {
    showPopUp_NVM(event.data);
  };

  eventSource.onerror = function (event) {
    eventSource.close();
    if (eventSource.readyState === 2) {
      console.log("Event source closed.");
      isEventSourceClosed = true;
      checkAndDownload();
    }
  };

  async function checkAndDownload() {
    if (isEventSourceClosed) {
      await download_button(path, fileName);
    }
  }
}

async function download_button(path, filename) {
  try {
    const response = await fetch("/download", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ path, filename }),
    });

    if (response.ok) {
      // Create and append the download button
      const downloadButton = document.createElement("button");
      downloadButton.style.marginTop = "1.5rem";
      downloadButton.innerText = "Download";
      downloadButton.onclick = async () => {
        const blob = await response.blob();
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        a.remove();
        window.URL.revokeObjectURL(url);
      };
      document.getElementById("terminal_output").appendChild(downloadButton);
    } else {
      const errorData = await response.json();
      console.error("Download failed:", errorData.error);
    }
  } catch (error) {
    console.error("Error checking file:", error);
  }
}

async function CCBRead() {
  const modal = document.getElementById("read_tools_overlay_ccb");

  // Open the modal
  modal.setAttribute("open", "true");

  // Wait for the modal to close
  await new Promise((resolve) => {
    const handler = () => {
      modal.removeEventListener("modalCloseRequest", handler);
      resolve();
    };
    modal.addEventListener("modalCloseRequest", handler);
  });

  // After modal is closed, do something async
  document
    .getElementById("read_tools_overlay_ccb")
    .setAttribute("open", "false");
}

async function runCCBScript() {
  const fileName = document.getElementById("CCB_file-name").value;
  document
    .getElementById("read_tools_overlay_ccb")
    .setAttribute("open", "false");
  const path = "Tools/host_boot_tools/NVM_Utils";
  const script = "./ccb-read.sh";

  const response = await fetch("/run_shell_script", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ version: fileName, script: script }),
  });

  if (!response.ok) {
    console.error("Error starting shell script:", response.statusText);
    return;
  }
  const eventSource = new EventSource("/run-script");
  isEventSourceClosed = false;

  terminalOutput.textContent = ""; // Clear previous output
  document.getElementById("terminal_modal").setAttribute("open", "true");
  terminalOutput.textContent = "Reading CCB ...\n\n";

  eventSource.onmessage = function (event) {
    showPopUp_NVM(event.data);
  };

  eventSource.onerror = function (event) {
    eventSource.close();
    if (eventSource.readyState === 2) {
      console.log("Event source closed.");
      isEventSourceClosed = true;
      checkAndDownload();
    }
  };

  async function checkAndDownload() {
    if (isEventSourceClosed) {
      await download_button(path, fileName);
    }
  }
}

// close modals on close requestread_tools_overlay_nvm
document
  .getElementById("terminal_modal")
  .addEventListener("modalCloseRequest", () => {
    if (isEventSourceClosed) {
      document.getElementById("terminal_modal").setAttribute("open", "false");
    } else {
      alert("Can't close the terminal. Please wait until process complete !");
    }
  });

// setting up the wifi
async function showWifiSetupForm() {
  const modal = document.getElementById("wifisetup_modal");

  // Open the modal
  modal.setAttribute("open", "true");

  // Wait for the modal to close
  await new Promise((resolve) => {
    const handler = () => {
      modal.removeEventListener("modalCloseRequest", handler);
      resolve();
    };
    modal.addEventListener("modalCloseRequest", handler);
  });

  // After modal is closed, do something async
  document.getElementById("wifisetup_modal").setAttribute("open", "false");
}

// **************************** Setup Wifi ********************************* //

async function WifiRefresh() {
  await getWifiUsername();
  await networkStatus();
}

async function getWifiUsername() {
  const getUsername = document.getElementById("wifi_username");

  try {
    const response = await fetch("/get-username");
    const result = await response.json();

    getUsername.textContent = result.username;
    getUsername.style.fontWeight = "600";
  } catch (error) {
    console.error("Error fetching username:", error);
  }
}

async function networkStatus() {
  const networkstatus = document.getElementById("network_status");

  try {
    const response = await fetch("/network-status");
    const result = await response.json();

    if (result.status) {
      networkstatus.textContent = "Connected";
      networkstatus.style.fontWeight = "bold";
      networkstatus.style.color = "green";
    } else {
      networkstatus.textContent = "Not Connected";
      networkstatus.style.fontWeight = "bold";
      networkstatus.style.color = "red";
    }
  } catch (error) {
    console.error("Error in fetching network status:", error);
  }
}

async function setupWifi() {
  const username = document.getElementById("username").value;
  const password = document.getElementById("password").value;
  const wifiStatusDiv = document.getElementById("wifiStatus");
  const wifisetupdis = document.getElementById("wifisetup_modal");
  wifisetupdis.setAttribute("open", "false");
  showOverlay("Setting up WiFi...");

  try {
    const response = await fetch("/setup-wifi", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ username, password }),
    });
    const result = await response.json();
    showOverlay(result.message);
    await delay(1000);
  } catch (error) {
    console.error("Error setting up WiFi:", error);
    showOverlay("Error setting up WiFi");
    await delay(1000);
  }
  hideOverlay();
}

// ********************************************* change to ecm ************************************************** /

async function changeToECM() {
  let osType;

  const platform = navigator.platform.toLowerCase();
  const userAgent = navigator.userAgent.toLowerCase();
  const networkOverlayContent = document.getElementById(
    "network-overlay-content"
  );
  const networkOverlay = document.getElementById("network-overlay");

  if (platform.includes("win")) {
    osType = "windows";
    console.log(osType);
  } else if (platform.includes("linux") || userAgent.includes("ubuntu")) {
    osType = "ubuntu";
    console.log(osType);
  }
  try {
    const response = await fetch("/Change-Network", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ value: osType }),
    });

    if (!response.ok) {
      console.error("Failed to execute the script");
      return;
    }

    const get_response = await fetch("/Change-Network-GET");

    if (!get_response.ok) {
      throw new Error("Failed to execute script");
    }

    const result = await get_response.json();

    console.log("Success:", result.message);
    networkOverlayContent.innerHTML = result.message;
    networkOverlay.setAttribute("open", "true");

    // close network modal

    networkOverlay.addEventListener("modalCloseRequest", () => {
      networkOverlay.setAttribute("open", "false");
    });
  } catch (error) {
    console.error("Fetch error:", error);
  }
}
