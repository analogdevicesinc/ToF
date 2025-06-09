// *********************************************************************  ADSD3500 section *************************************************************************/

function showOverlay(message = "Processing, please wait...") {
  const overlay = document.getElementById("overlay");
  const overlayText = document.getElementById("overlay-text");
  overlayText.textContent = message;
  overlay.style.display = "flex";
}

function hideOverlay() {
  document.getElementById("overlay").style.display = "none";
}

async function updateListbox() {
  try {
    let response = await fetch("/get-workspace");
    let data = await response.json();

    let listbox = document.getElementById("output");
    for (let i = 0; i < listbox.options.length; i++) {
      if (listbox.options[i].text === data.workspace) {
        listbox.options[i].selected = true;
        break;
      }
    }
  } catch (error) {
    console.error("Error fetching workspace:", error);
  }
}

async function updateIniListbox() {
  try {
    let response = await fetch("/get-workspace");
    let data = await response.json();

    const displayElement = document.getElementById("currentWorkspaceDisplay");
    if (data.workspace) {
      displayElement.textContent = "Current Workspace: " + data.workspace;
    } else {
      displayElement.textContent = "Current Workspace: Not Selected";
    }

    let listbox = document.getElementById("output");
    for (let i = 0; i < listbox.options.length; i++) {
      if (listbox.options[i].text === data.workspace) {
        listbox.options[i].selected = true;
        break;
      }
    }
  } catch (error) {
    console.error("Error fetching workspace:", error);
  }
}

function setStatusMessage(message) {
  document.getElementById("progressBar").setAttribute("label", message);
}

async function runScript() {
  try {
    console.log("Running script to refresh output list...");
    const response = await fetch("/execute-post-script");

    if (!response.ok) {
      throw new Error("Failed to execute post script");
    }

    const outputLines = await response.json();
    const outputElement = document.getElementById("output");

    outputElement.innerHTML = "";

    outputLines.forEach((line) => {
      const option = document.createElement("option");
      option.text = line;
      outputElement.add(option);
    });

    // Add one more option for the not selected any version
    const noChange = document.createElement("option");
    noChange.text = `No Change`;
    outputElement.add(noChange);

    await updateListbox();
    await update_version(selected_mode);
    await loadCurrentFirmware();
  } catch (error) {
    console.error("Error running script to refresh list:", error);
    setStatusMessage("Error refreshing list.");
  }
}

async function update_version(selected_mode = "Not set") {
  const OpreationMode = document.getElementById("op_mode");
  const CurrentWorkspace = document.getElementById("current_workspace");
  let sdk_response = await fetch("/get-workspace");
  let sdk_data = await sdk_response.json();

  CurrentWorkspace.innerHTML = sdk_data.workspace;
  OpreationMode.innerHTML = "Not set";
}

async function switchWorkspace() {
  const listbox = document.getElementById("output");
  const selectedOption = listbox.options[listbox.selectedIndex];

  if (selectedOption) {
    const selectedEntry = selectedOption.text;
    showOverlay("Switching workspace...");

    try {
      const response = await fetch("/switch-workspace", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ entry: selectedEntry }),
      });

      const result = await response.json();
      if (response.ok) {
        const displayElement = document.getElementById("current_workspace");
        displayElement.textContent = selectedEntry;
      } else {
        console.error("Error switching workspace:", result.error);
      }
    } catch (error) {
      console.error("Error:", error);
    } finally {
      hideOverlay();
    }
  } else {
    alert("Please select an entry in the listbox first.");
  }
}

document.addEventListener("DOMContentLoaded", async () => {
  const applyButton = document.getElementById("apply");
  const outputElement = document.getElementById("output");
  const firmwareSelect = document.getElementById("firmware_version");
  const operatingMode = document.getElementById("ros_mode");

  applyButton.addEventListener("click", () => {
    // change the workspace
    const selectedVersion = outputElement.value;
    console.log(`Selected Workspace Version: ${selectedVersion}`);
    if (selectedVersion !== "No Change") {
      switchWorkspace();
    }

    // chage the oprating mode
    const selectedOpratingVersion = operatingMode.value;
    console.log(`Selected Operating mode: ${selectedOpratingVersion}`);
    if (selectedVersion !== "No Change") {
      update_version(selectedOpratingVersion);
    }


    // flash the firmware
    const selectedFirmwareVersion = firmwareSelect.value;
    console.log(`Selected Firmware Version: ${selectedFirmwareVersion}`);
    if (selectedFirmwareVersion !== "No Change") {
      flashFirmware(selectedFirmwareVersion);
    }
  });
});

// ********************************************************* for Select Mode of Operation **************************************************************************************/

// Example list of workspace versions
const mode_of_operation = ["Legacy", "ROS-1", "ROS-2", "No Change"];
let selected_mode = "Not Set"; // Use 'let' so it can be updated

document.addEventListener("DOMContentLoaded", () => {
  const selectElement = document.getElementById("ros_mode");

  // Populate the select element
  mode_of_operation.forEach((version) => {
    const option = document.createElement("option");
    option.value = version;
    option.textContent = version;
    if (version === selected_mode) {
      option.selected = true;
    }
    selectElement.appendChild(option);
  });

  // Add change event listener to update selected_mode
  selectElement.addEventListener("change", () => {
    selected_mode = selectElement.value || "not set";
    console.log("Selected mode:", selected_mode);
  });
});


// ******************************************************** for firmware update  **************************************************************************************/
// Function to get current firmware version

const terminalOutput = document.getElementById("terminal_output");

async function loadCurrentFirmware() {
  const firmwareVersion = document.getElementById("firm_version");
  const firmwareVersionWarn = document.getElementById("device_info");
  let this_firmware = 0;
  try {
    const response = await fetch("/current_firmware");
    const data = await response.json();
    this_firmware = data.current_firmware;
    firmwareVersion.textContent = `${data.current_firmware}`;
  } catch (error) {
    console.error("Error loading current firmware:", error);
  }

  //  check the compatible version
  try {
    const response = await fetch("/check_firmware");
    const data = await response.json();
    if (this_firmware !== data.version) {
      firmwareVersionWarn.style.color = "red";
      firmwareVersionWarn.textContent = `Warning! Firmware Version : ${data.version} is recommended.`;
    } else {
      firmwareVersionWarn.textContent = `Refresh to get latest version info`;
    }
  } catch (error) {
    console.error("Error loading current firmware:", error);
  }
}

let isfirmwareflashing = false;

function showPopUp(message) {
  const packetRegex = /Packet number: (\d+) \/ (\d+)/;
  const waitingRegex = /Waiting for (\d+) seconds/;

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
      outputLines[i] = `Packet number: ${currentPacket} / ${totalPackets}`;
      updated = true;
    } else if (
      waitingRegex.test(outputLines[i]) &&
      waitingRegex.test(message)
    ) {
      // Update Waiting for seconds line
      const match = message.match(waitingRegex);
      const seconds = match[1];
      outputLines[i] = `Waiting for ${seconds} seconds`;
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

async function runShellScript(version) {
  const response = await fetch("/run_shell_script", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ version: version }),
  });

  if (!response.ok) {
    console.error("Error starting shell script:", response.statusText);
    return;
  }

  const eventSource = new EventSource("/events");
  isfirmwareflashing = true;

  terminalOutput.textContent = ""; // Clear previous output
  document.getElementById("terminal_modal").setAttribute("open", "true");
  terminalOutput.textContent = "Firmware Flashing ...\n\n";

  eventSource.onmessage = function (event) {
    const data = event.data;

    // Check for special exit code message
    if (data.startsWith("__EXIT_CODE__:")) {
      const exitCode = parseInt(data.split(":")[1]);

      if (exitCode === 0) {
        alert("Firmware flashing completed successfully!");
      } else {
        alert("Firmware flashing failed.");
      }
      return;
    }

    // Otherwise, show regular output
    showPopUp(data);
  };

  eventSource.onerror = function (event) {
    eventSource.close();
    isfirmwareflashing = false;
  };
}

document
  .getElementById("terminal_modal")
  .addEventListener("modalCloseRequest", () => {
    if (isfirmwareflashing) {
      alert(
        "Firmware flashing is still in progress. Please wait until it completes."
      );
      document.getElementById("terminal_modal").setAttribute("open", "true");
    } else {
      document.getElementById("terminal_modal").setAttribute("open", "false");
    }
  });

async function flashFirmware(option) {
  if (option) {
    // Show confirmation dialog

    const userConfirmed = confirm(
      "Do you agree to proceed with the firmware update? This action cannot be undone."
    );
    if (userConfirmed) {
      runShellScript(option);
    } else {
      document.getElementById("terminal_modal").setAttribute("open", "true");
      terminalOutput.textContent = "Firmware update canceled by user.";
    }
  } else {
    terminalOutput.textContent = "Please select a firmware version";
  }
}

document.addEventListener("DOMContentLoaded", async () => {
  const firmwareSelect = document.getElementById("firmware_version");

  // Function to read firmware versions from a directory
  async function loadFirmwareVersions() {
    try {
      const response = await fetch("/list_firmware_versions");
      if (!response.ok) {
        throw new Error("Failed to execute post script");
      }
      const data = await response.json();
      data.forEach((version) => {
        const option = document.createElement("option");
        option.text = version;
        firmwareSelect.appendChild(option);
      });

      const notSelected = document.createElement("option");
      notSelected.text = `No Change`;
      firmwareSelect.appendChild(notSelected);
    } catch (error) {
      console.error("Error loading firmware versions:", error);
    }
  }

  // Load firmware versions and current firmware on page load
  await loadFirmwareVersions();
  await loadCurrentFirmware();
});

// ********************************* File Upload *********************************** //

async function uploadFileInChunks() {
  const fileInput = document.getElementById("fileInput");
  const progressBar = document.getElementById("progressBar");
  const file = fileInput.files[0];

  if (file) {
    showOverlay("Uploading file...");
    showProgressElements();
    setStatusMessage("Uploading...");

    const chunkSize = 5 * 1024 * 1024;
    const totalChunks = Math.ceil(file.size / chunkSize);

    // progressBar.max = 100;

    for (let chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
      const chunk = file.slice(
        chunkIndex * chunkSize,
        (chunkIndex + 1) * chunkSize
      );
      const formData = new FormData();
      formData.append("file", chunk);
      formData.append("chunkIndex", chunkIndex);
      formData.append("totalChunks", totalChunks);

      try {
        const response = await fetch("/upload-chunk", {
          method: "POST",
          body: formData,
        });
        const result = await response.json();
        ro_access = result;

        const percentComplete = Math.round(
          ((chunkIndex + 1) / totalChunks) * 100
        );
        // progressBar.value = percentComplete;
        progressBar.setAttribute("value", percentComplete);
        // progressPercentage.textContent = `${percentComplete}%`;
      } catch (error) {
        console.error("Error uploading chunk", chunkIndex, error);
        alert("An error occurred during file upload.");
        setStatusMessage("Upload failed.");
        hideProgressElements();
        hideOverlay();
        return;
      }
    }

    await access();
    await startBackgroundUnzip(totalChunks);
    fileInput.value = "";
  } else {
    alert("Please select a file to upload.");
  }
}

function showProgressElements() {
  document.getElementById("progressBar").style.display = "block";
}

function hideProgressElements() {
  document.getElementById("progressBar").style.display = "none";
}

let ro_access = false;

async function access() {
  try {
    const response = await fetch("/ro-access", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ value: ro_access }),
    });
  } catch (error) {
    console.error("Error in sending access", error);
  }
}

async function startBackgroundUnzip(totalChunks) {
  setStatusMessage("Starting background unzip...");

  const formData = new FormData();
  formData.append("totalChunks", totalChunks);

  try {
    const response = await fetch("/complete-upload", {
      method: "POST",
      body: formData,
    });

    const result = await response.json();

    if (!response.ok) {
      setStatusMessage("Unzip initiation failed: " + result.error);
      alert("Error: " + result.error);
      hideProgressElements();
      hideOverlay();
    } else {
      setStatusMessage("Unzipping in progress...");
      pollUnzipStatus();
    }
  } catch (error) {
    console.error("Error starting unzip", error);
    alert("An error occurred while initiating unzip.");
    setStatusMessage(
      "Unzip initiation failed due to a network or server error."
    );
    hideProgressElements();
    hideOverlay();
  }
}

async function pollUnzipStatus() {
  try {
    console.log("Polling for unzip status...");
    const response = await fetch("/unzip-status");

    if (!response.ok) {
      throw new Error("Network response was not ok");
    }

    const status = await response.json();
    console.log("Received unzip status:", status);

    if (status.status === "completed") {
      alert("Upload Successfull !!");
      hideProgressElements();
      hideOverlay();
    } else if (status.status === "failed") {
      alert(status.validation);
      hideProgressElements();
      hideOverlay();
    } else if (status.status === "done") {
      setStatusMessage("Process completed and cleaned up.");
      hideProgressElements();
      hideOverlay();
    } else if (status.status === "unpacking") {
      setStatusMessage("Unzipping in progress...");
      setTimeout(pollUnzipStatus, 2000);
    } else if (status.status === "no_task") {
      setStatusMessage("No unzip task found.");
      setTimeout(pollUnzipStatus, 2000);
    } else {
      setStatusMessage("Unexpected status received, retrying...");
      setTimeout(pollUnzipStatus, 2000);
    }
  } catch (error) {
    console.error("Error polling unzip status:", error);
    setStatusMessage("Error polling unzip status, retrying...");
    setTimeout(pollUnzipStatus, 2000);
  }
}
