let timerInterval;  // Store the timer interval ID
let elapsedTime = 0;  // Track elapsed time in seconds

// Timer functions to show elapsed time during processes
function startTimer() {
    elapsedTime = 0;  // Reset elapsed time
    document.getElementById('timerDisplay').textContent = `Time elapsed: ${elapsedTime}s`;
    
    // Update timer every second
    timerInterval = setInterval(() => {
        elapsedTime++;
        document.getElementById('timerDisplay').textContent = `Time elapsed: ${elapsedTime}s`;
    }, 1000);
}

function stopTimer() {
    clearInterval(timerInterval);  // Stop the timer
}

function showOverlay(message = "Processing, please wait...") {
    const overlay = document.getElementById('overlay');
    const overlayText = document.getElementById('overlay-text');
    overlayText.textContent = message;
    overlay.style.display = 'flex';
}

function hideOverlay() {
    document.getElementById('overlay').style.display = 'none';
}

function openTab(event, tabId) {
    // Hide all content sections
    const contents = document.querySelectorAll('.content');
    contents.forEach(content => {
        content.style.display = 'none';
        content.classList.remove('active');
    });

    // Remove active class from all tabs
    const tabs = document.querySelectorAll('.tab');
    tabs.forEach(tab => {
        tab.classList.remove('active');
    });

    // Show the selected tab's content and set it as active
    document.getElementById(tabId).style.display = 'block';
    document.getElementById(tabId).classList.add('active');
    event.currentTarget.classList.add('active');
}


document.addEventListener('DOMContentLoaded', function() {
    updateDisplayedTimes();
});

async function updateSystemTab() {
    try {
        const response = await fetch('/server-time');
        const data = await response.json();
        const serverTimeUtc = data.server_time_utc;

        // Convert server time to browser's local timezone
        const serverTime = new Date(serverTimeUtc + ' UTC');
        const browserTimeZone = Intl.DateTimeFormat().resolvedOptions().timeZone;
        const serverTimeInBrowserTZ = serverTime.toLocaleString('en-US', {
            timeZone: browserTimeZone,
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: true,
            timeZoneName: 'short'
        });

        // Split the date and time
        const [date, time] = serverTimeInBrowserTZ.split(', ');

        // Update the table cells
        document.getElementById('evalDate').textContent = date;
        document.getElementById('evalTime').textContent = time;
      } catch (error) {
        console.error('Error fetching server time:', error);
        document.getElementById('evalDate').textContent = 'Error';
        document.getElementById('evalTime').textContent = 'Error';
      }

    const browserTime = new Date().toLocaleString('en-US', {
        timeZoneName: 'short',
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: true
    });
    // Split the date and time
    const [date, time] = browserTime.split(', ');

    // Update the table cells
    document.getElementById('browserDate').textContent = date;
    document.getElementById('browserTime').textContent = time;

}

function updateDisplayedTimes() {
    updateSystemTab();
}

async function setServerTime() {
    const browserTime = new Date();
    const timeZoneName = Intl.DateTimeFormat().resolvedOptions().timeZone; // Full timezone name
    const timeZoneAbbr = new Date().toLocaleTimeString('en-us', { timeZoneName: 'short' }).split(' ')[2]; // Short name like EST/EDT
    
    try {
        const response = await fetch('/set-server-time', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                browser_time: browserTime.toLocaleString('en-US', { timeZone: "Asia/Kolkata" }), 
                time_zone_name: timeZoneName
            })
        });

        console.log(`TimeZone Name: ${timeZoneName}`);
        console.log(`TimeZone Abbreviation: ${timeZoneAbbr}`);


        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Failed to set server time');
        }

        const result = await response.json();
        alert(result.message);
        updateSystemTab();
    } catch (error) {
        console.error('Error setting server time:', error);
        alert('Error setting server time: ' + error.message);
    }
}

async function updateListbox() {
    try {
        let response = await fetch('/get-workspace');
        let data = await response.json();

        let listbox = document.getElementById('output');
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
        let response = await fetch('/get-workspace');
        let data = await response.json();

        const displayElement = document.getElementById('currentWorkspaceDisplay');
        if (data.workspace) {
            displayElement.textContent = "Current Workspace: " + data.workspace;
        } else {
            displayElement.textContent = "Current Workspace: Not Selected";
        }

        let listbox = document.getElementById('output');
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

async function runScript() {
    try {
        console.log("Running script to refresh output list...");
        const response = await fetch('/execute-post-script');

        if (!response.ok) {
            throw new Error("Failed to execute post script");
        }

        const outputLines = await response.json();
        const outputElement = document.getElementById('output');
        
        outputElement.innerHTML = '';
        
        outputLines.forEach(line => {
            const option = document.createElement('option');
            option.text = line;
            outputElement.add(option);
        });

        // Add one more option for the not selected any version
        const noChange = document.createElement('option');
        noChange.text = `No Change`;
        outputElement.add(noChange);

        console.log("List refreshed with new data:", outputLines);

        await updateListbox();
        await update_version();
        await loadCurrentFirmware();
        await WifiRefresh();
    } catch (error) {
        console.error("Error running script to refresh list:", error);
        setStatusMessage("Error refreshing list.");
    }
}

function setStatusMessage(message) {
    document.getElementById('statusMessage').textContent = message;
}

function showProgressElements() {
    document.getElementById('progressBar').style.display = 'block';
    document.getElementById('progressPercentage').style.display = 'block';
    document.getElementById('timerDisplay').style.display = 'block';
}

function hideProgressElements() {
    document.getElementById('progressBar').style.display = 'none';
    document.getElementById('progressPercentage').style.display = 'none';
    document.getElementById('timerDisplay').style.display = 'none';
}


async function uploadFileInChunks() {
    const fileInput = document.getElementById('fileInput');
    const progressBar = document.getElementById('progressBar');
    const progressPercentage = document.getElementById('progressPercentage');
    const file = fileInput.files[0];

    if (file) {
        showOverlay("Uploading file...");
        showProgressElements();
        setStatusMessage("Uploading...");
        startTimer();

        const chunkSize = 5 * 1024 * 1024;
        const totalChunks = Math.ceil(file.size / chunkSize);

        progressBar.max = 100;

        for (let chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
            const chunk = file.slice(chunkIndex * chunkSize, (chunkIndex + 1) * chunkSize);
            const formData = new FormData();
            formData.append('file', chunk);
            formData.append('chunkIndex', chunkIndex);
            formData.append('totalChunks', totalChunks);

            try {
                const response = await fetch('/upload-chunk', {
                    method: 'POST',
                    body: formData
                });
                const result = await response.json();
                console.log(result.message);

                const percentComplete = Math.round(((chunkIndex + 1) / totalChunks) * 100);
                progressBar.value = percentComplete;
                progressPercentage.textContent = `${percentComplete}%`;

            } catch (error) {
                console.error('Error uploading chunk', chunkIndex, error);
                alert("An error occurred during file upload.");
                setStatusMessage("Upload failed.");
                stopTimer();
                hideProgressElements();
                hideOverlay();
                return;
            }
        }
        
        await startBackgroundUnzip(totalChunks);
        fileInput.value = '';
    } else {
        alert("Please select a file to upload.");
    }
}

async function startBackgroundUnzip(totalChunks) {
    setStatusMessage("Starting background unzip...");

    const formData = new FormData();
    formData.append('totalChunks', totalChunks);

    try {
        const response = await fetch('/complete-upload', {
            method: 'POST',
            body: formData
        });
        
        const result = await response.json();

        if (!response.ok) {
            setStatusMessage("Unzip initiation failed: " + result.error);
            alert("Error: " + result.error);
            hideProgressElements();
            stopTimer();
            hideOverlay();
        } else {
            setStatusMessage("Unzipping in progress...");
            pollUnzipStatus();
        }
    } catch (error) {
        console.error('Error starting unzip', error);
        alert("An error occurred while initiating unzip.");
        setStatusMessage("Unzip initiation failed due to a network or server error.");
        hideProgressElements();
        stopTimer();
        hideOverlay();
    }
}

async function pollUnzipStatus() {
    try {
        console.log("Polling for unzip status...");
        const response = await fetch('/unzip-status');
        
        if (!response.ok) {
            throw new Error("Network response was not ok");
        }
        
        const status = await response.json();
        console.log("Received unzip status:", status);
        
        if (status.status === "completed") {
            setStatusMessage("Unzipping and processing completed successfully!");
            stopTimer();
            hideProgressElements();
            hideOverlay();
            
        } else if (status.status === "failed") {
            setStatusMessage("Process failed: " + (status.error || "Unknown error"));
            stopTimer();
            hideProgressElements();
            hideOverlay();
            
        } else if (status.status === "done") {
            setStatusMessage("Process completed and cleaned up.");
            stopTimer();
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

async function switchWorkspace() {
    const listbox = document.getElementById('output');
    const selectedOption = listbox.options[listbox.selectedIndex];

    if (selectedOption) {
        const selectedEntry = selectedOption.text;
        showOverlay("Switching workspace...");

        try {
            const response = await fetch('/switch-workspace', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ entry: selectedEntry })
            });

            const result = await response.json();
            if (response.ok) {
                const displayElement = document.getElementById('current_workspace');
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

function showCountdownOverlay(seconds, action) {
    const overlayContent = document.querySelector('.overlay-content');
    document.getElementById('overlay').style.display = 'flex';

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
    fetch('/reboot-system', { method: 'POST' })
        .then(response => {
            if (!response.ok) {
                console.error("Failed to reboot the system.");
            }
        })
        .catch(error => {
            console.error("Error while attempting to reboot:", error);
        });

    document.querySelector('.overlay-content').innerHTML = `<div class="spinner"></div><p>System is rebooting. Reconnecting soon...</p>`;
    pollServerReconnect();
}

function initiatePowerDown() {
    fetch('/power-down-system', { method: 'POST' })
        .then(response => {
            if (!response.ok) {
                console.error("Failed to power down the system.");
            }
        })
        .catch(error => {
            console.error("Error while attempting to power down:", error);
        });

    document.querySelector('.overlay-content').innerHTML = `<div class="spinner"></div><p>System is powering down...</p>`;
}

function pollServerReconnect() {
    const reconnectInterval = setInterval(async () => {
        try {
            const response = await fetch('/');
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

async function runService() {
    const outputElement = document.getElementById('serviceOutput');
    outputElement.textContent = "Running service...";

    try {
        const response = await fetch('/run-service-backup-nvm', { method: 'POST' });

        if (!response.ok) {
            throw new Error("Failed to run the service");
        }

        const data = await response.json();
        outputElement.textContent = data.output;
    } catch (error) {
        console.error("Error running service:", error);
        outputElement.textContent = "Unable to run service.";
    }
}

function runServiceBackupNVM() {
    const pollingInterval = 2000; // Check every 2 seconds

    // Show overlay with spinner while polling is active
    showOverlay("Monitoring file sizes, please wait...");

    function checkFileSizes() {

        fetch('/check-file-sizes')
            .then(response => response.json())
            .then(data => {
                if (data.error) {
                    console.error("Error checking file sizes:", data.error);
                    hideOverlay(); // Hide overlay on error
                    return;
                }

                const { ccb_size, nvm_size, is_complete } = data;
                console.log(`CCB File: ${ccb_size} bytes, NVM File: ${nvm_size} bytes`);

                // Update the UI to show file sizes
                document.getElementById('fileSizes').textContent =
                    `CCB File: ${(ccb_size / (1024)).toFixed(2)} KB, ` +
                    `NVM File: ${(nvm_size / (1024)).toFixed(2)} KB`;

                // Stop polling if both files meet the size threshold
                if (is_complete) {
                    console.log("Both files have reached the size threshold. Stopping.");
                    clearInterval(pollingId);
                    hideOverlay(); // Hide overlay on error

                    // Optional: Show a success message
                    document.getElementById('statusMessage').textContent = "Both files have reached the required size!";
                }
            })
            .catch(error => console.error("Error during polling:", error));
            hideOverlay(); // Hide overlay on error
    }

    // Start polling
    const pollingId = setInterval(checkFileSizes, pollingInterval);
    runService();
    checkFileSizes(); // Initial check
}

// ********************************************************** change the color on selecting the section **************************************************/
document.addEventListener('DOMContentLoaded', function() {
    const navLinks = document.querySelectorAll('.nav_link a');

    navLinks.forEach(link => {
        link.addEventListener('click', function(event) {

            // Remove active class from all links
            navLinks.forEach(link => link.parentElement.classList.remove('active'));

            // Add active class to the clicked link
            this.parentElement.classList.add('active');
        });
    });
});

// setting up the wifi
function showWifiSetupForm() {
    document.getElementById('wifisetup_section').style.display = 'flex';
}

function closeWifi() {
    document.getElementById('wifisetup_section').style.display = 'none';
}

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

async function WifiRefresh() {
    await getWifiUsername();
    await networkStatus();
}

async function getWifiUsername() {
    const getUsername = document.getElementById('wifi_username');

    try {
        const response = await fetch('/get-username');
        const result = await response.json();

        getUsername.innerHTML = result.username;
        getUsername.style.display = 'block-inline';
        getUsername.style.fontWeight = '600';

    } catch (error) {
        console.error('Error fetching username:', error);
    }
}

async function networkStatus() {
    const networkstatus = document.getElementById('network_status');

    try {
        const response = await fetch('/network-status');
        const result = await response.json();

        if (result.status) {
            networkstatus.innerHTML = 'Connected';
            networkstatus.style.display = 'block-inline';
            networkstatus.style.fontWeight = 'bold';
            networkstatus.style.color = 'green';
        } else {
            networkstatus.innerHTML = 'Not Connected';
            networkstatus.style.display = 'block-inline';
            networkstatus.style.fontWeight = 'bold';
            networkstatus.style.color = 'red';
        }

    } catch (error) {
        console.error('Error in fetching network status:', error);
    }
}

async function setupWifi() {
    const username = document.getElementById('username').value;
    const password = document.getElementById('password').value;
    const wifiStatusDiv = document.getElementById('wifiStatus');
    const wifisetupdis = document.getElementById('wifisetup_section');
    wifisetupdis.style.display = 'none';
    showOverlay('Setting up WiFi...');

    try {
        const response = await fetch('/setup-wifi', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ username, password })
        });
        const result = await response.json();
        showOverlay(result.message);
        await delay(1000);
         
    } catch (error) {
        console.error('Error setting up WiFi:', error);
       showOverlay('Error setting up WiFi');
       await delay(1000);
    }
    hideOverlay();
    
}

// ******************************************************************** show the latest release *****************************************************/

async function fetchLatestRelease() {
    const response = await fetch('https://api.github.com/repos/analogdevicesinc/ToF/releases/latest');
    const data = await response.json();
    return data;
}

async function showLatestRelease() {
    const versionElement = document.getElementById('latest-version');
    const downloadLinkElement = document.getElementById('download-link');
    const releaseDateElement = document.getElementById('release-date');
    try {
        const latestRelease = await fetchLatestRelease();
        const latestVersion = latestRelease.tag_name;
        const releaseDate = new Date(latestRelease.published_at).toLocaleDateString(); // Extract and format release date

        // Determine the user's operating system
        const userAgent = navigator.userAgent;
        const isWindows = userAgent.indexOf('Win') > -1;
        const isLinux = userAgent.indexOf('Linux') > -1;

        // Find the appropriate asset for the user's OS
        let downloadUrl = '';
        latestRelease.assets.forEach(asset => {
            if (isWindows && asset.name.includes('.exe')) {
                downloadUrl = asset.browser_download_url;
            } else if (isLinux && asset.name.includes('.sh')) {
                downloadUrl = asset.browser_download_url;
            }
        });

        versionElement.textContent = `Latest Release: ${latestVersion}`;
        downloadLinkElement.href = downloadUrl;
        downloadLinkElement.textContent = 'Download';
        releaseDateElement.textContent = `Release Date: ${releaseDate}`; // Display release date
    } catch (error) {
        console.error('Error fetching latest release:', error);
        versionElement.textContent = 'Error fetching latest release';
        downloadLinkElement.textContent = '';
        releaseDateElement.textContent = ''; // Clear release date on error
    }
}

document.addEventListener('DOMContentLoaded', showLatestRelease);

// ********************************************** open the readme files for different contents *************************************************************/

async function loadMarkdown(directory, fileName,id) {
    try {
        // Fetch the Markdown content from the server
        const response = await fetch(`/get-markdown?directory=${encodeURIComponent(directory)}&file=${encodeURIComponent(fileName)}`);
        const data = await response.json();

        const contentElement = document.getElementById(id);

        if (data.error) {
            contentElement.textContent = data.error;
        } else {
            const markdownContent = data.markdown;
            const htmlContent = marked.parse(markdownContent);
            contentElement.innerHTML = htmlContent;
            contentElement.classList.add('active');
            contentElement.classList.remove('hidden');
        }
    } catch (error) {
        console.error('Error fetching Markdown:', error);
        document.getElementById('markdownContent').textContent = 'Failed to load content.';
    }
}

document.addEventListener('DOMContentLoaded', function() {
    const links = document.querySelectorAll('.main_menu a');
    const sections = document.querySelectorAll('.markdown-body');

    links.forEach(link => {
        link.addEventListener('click', function(event) {
            event.preventDefault(); // Prevent default anchor behavior

            // Hide all sections
            sections.forEach(section => {
                section.classList.remove('active');
                section.classList.add('hidden');
            });

            // Show the clicked section
            const targetId = this.getAttribute('href').substring(1);
            const targetSection = document.getElementById(targetId);

            if (targetId === 'evalkit-readme-content') {

                loadMarkdown("web-1.0.0/static/docs","evalkit_readme.md","evalkit-readme-content");
            } else if (targetId === 'webinterface') {
                loadMarkdown("web-1.0.0/static/docs","webinterface.md","webinterface");
            }else {
                targetSection.classList.add('active');
                targetSection.classList.remove('hidden');
            }
        });
    });
});

// *********************************************************************  ToF section *************************************************************************/

async function update_version(){

    
    const OpreationMode = document.getElementById('op_mode');
    const CurrentWorkspace = document.getElementById('current_workspace');
    let sdk_response = await fetch('/get-workspace');
    let sdk_data = await sdk_response.json();

    CurrentWorkspace.innerHTML = sdk_data.workspace;
    OpreationMode.innerHTML = selected_mode;
}


document.addEventListener('DOMContentLoaded', async () => {
    const applyButton = document.getElementById('apply');
    const outputElement = document.getElementById('output');
    const firmwareSelect = document.getElementById('firmware_version');

    applyButton.addEventListener('click', () =>{

        // change the workspace
        const selectedVersion = outputElement.value;
        console.log(`Selected Workspace Version: ${selectedVersion}`);
        if (selectedVersion !== 'No Change') {
                switchWorkspace();
        }

        // flash the firmware
        const selectedFirmwareVersion = firmwareSelect.value;
        console.log(`Selected Firmware Version: ${selectedFirmwareVersion}`);
        if (selectedFirmwareVersion !== 'No Change') {
            flashFirmware(selectedFirmwareVersion);
        }

    });
});




// ********************************************************* for Select Mode of Operation **************************************************************************************/

// Example list of workspace versions
const mode_of_operation = ["ROS-1", "ROS-2", "legacy"];
const selected_mode = "ROS-2";

// Populate the select element with versions
document.addEventListener('DOMContentLoaded', async () => {

const selectElement = document.getElementById('ros_mode');

mode_of_operation.forEach(version => {
    const option = document.createElement('option');
    option.value = `${version}`;
    option.innerHTML = `${version}`; 
    if(version === selected_mode)
    {
        option.selected = true;
    }
    selectElement.appendChild(option);
});

// Event listener for changing the workspace version
selectElement.addEventListener('change', (event) => {
    const selectedVersion = event.target.value;
    console.log(`Selected Mode of Operation: ${selectedVersion}`);
});

});

// ******************************************************** for firmware update  **************************************************************************************/
// Function to get current firmware version

const terminalOutput = document.getElementById('terminal_output');

async function warnCurrentFirmware(){
    

}
async function loadCurrentFirmware() {
    const firmwareVersion = document.getElementById('firm_version');
    const firmwareVersionWarn = document.getElementById('firm_version_warning');
    let this_firmware=0;
    try {
        const response = await fetch('/current_firmware');
        const data = await response.json();
        this_firmware=data.current_firmware
        firmwareVersion.textContent = `${data.current_firmware}`;
    } catch (error) {
        console.error('Error loading current firmware:', error);
    }

    //  check the compatible version
    try {
        const response = await fetch('/check_firmware');
        const data = await response.json();
        if (this_firmware !== data.version){
            firmwareVersionWarn.innerHTML = `Warning! Version : ${data.version} is recommended.`;
        }else{
            firmwareVersionWarn.innerHTML = ``;
        }

    } catch (error) {
        console.error('Error loading current firmware:', error);
    }

}

function showPopUp(message) {
    const packetRegex = /Packet number: (\d+) \/ (\d+)/;
    const waitingRegex = /Waiting for (\d+) seconds/;
 
    // Split the current terminal output into lines
    const outputLines = terminalOutput.textContent.split('\n');
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
        } else if (waitingRegex.test(outputLines[i]) && waitingRegex.test(message)) {
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
    terminalOutput.textContent = outputLines.join('\n');
}

async function runShellScript(version) {
    
    const response = await fetch('/run_shell_script', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ version: version })
    });

    if (!response.ok) {
        console.error('Error starting shell script:', response.statusText);
        return;
    }

    const eventSource = new EventSource('/events');

    terminalOutput.textContent = ''; // Clear previous output
    document.getElementById('terminal_modal').style.display = 'block';
    terminalOutput.textContent = 'Firmware Flashing ...\n\n'; 

    eventSource.onmessage = function(event) {
        showPopUp(event.data);
    };

    eventSource.onerror = function(event) {
        eventSource.close();
    };
}

// Get the modal
var modal = document.getElementById('terminal_modal');

// Get the <span> element that closes the modal
var span = document.getElementsByClassName('close')[0];

// When the user clicks on <span> (x), close the modal
span.onclick = function() {
    modal.style.display = 'none';
}

// When the user clicks anywhere outside of the modal, close it
window.onclick = function(event) {
    if (event.target == modal) {
        modal.style.display = 'none';
    }
}

async function flashFirmware(option){
    
    if (option) {
        // Show confirmation dialog

        const userConfirmed = confirm("Do you agree to proceed with the firmware update? This action cannot be undone.");
        if (userConfirmed) {
            runShellScript(option);
        } else {
            document.getElementById('terminal_modal').style.display = 'block';
            terminalOutput.textContent = 'Firmware update canceled by user.';
        }
    } else {
        terminalOutput.textContent = 'Please select a firmware version';
    }
}

document.addEventListener('DOMContentLoaded', async () => {
    const firmwareSelect = document.getElementById('firmware_version');
    

    // Function to read firmware versions from a directory
    async function loadFirmwareVersions() {
        try {
            const response = await fetch('/list_firmware_versions');
            if (!response.ok) {
                throw new Error("Failed to execute post script");
            }
            const data = await response.json();
            data.forEach(version => {
                const option = document.createElement('option');
                option.text = version;
                firmwareSelect.appendChild(option);
            });

            const notSelected = document.createElement('option');
            notSelected.text = `No Change`;
            firmwareSelect.appendChild(notSelected);
        } catch (error) {
            console.error('Error loading firmware versions:', error);
        }
    }

    
    
    // Load firmware versions and current firmware on page load
    await loadFirmwareVersions();
    await loadCurrentFirmware();
});

// ***************************************************** System Tab ********************************************************************/

// ***************************************************** Modify Permission *************************************************************/

async function showPermissionOverlay(){

    const permissionOverlay = document.getElementById('permission-overlay');
    const permissionOverlayContent = document.getElementById('permission-overlay-content');

    try {

        const response = await fetch('/Change-Permission',{
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ value: 'view' })
        });

        if(!response.ok){
            console.error("Failed to execute the script");
            return;
        }

        const newEvent = await fetch('/Change-Permission-Events')

        if(!newEvent.ok){
            throw new Error("Failed to execute script");
        }

        const data = await newEvent.json();

        permissionOverlayContent.innerHTML = data.output;
        permissionOverlay.style.display = 'flex';

        // Close the overlay when clicking outside of it
        window.onclick = function(event) {
            if (event.target == permissionOverlay) {
                permissionOverlay.style.display = 'none';
            }
        }
        
    } catch (error) {
        console.error('Error in Viewing Permission:', error);
    }
}

async function modifyPermission(){

    try {

        const response = await fetch('/Change-Permission',{
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ value: 'modify' })
        });

        if(!response.ok){
            console.error("Failed to execute the script");
            return;
        }

        const newEvent = await fetch('/Change-Permission-Events')

        if(!newEvent.ok){
            throw new Error("Failed to execute script");
        }

        const data = await newEvent.json();

        const userConfirmed = confirm("Do you agree to " + data.output + "? If you agree, the system will restart to apply these changes.");
        if (userConfirmed) {
            startRebootCountdown();
        }else{
            const retreat_back = await fetch('/Change-Permission-Events')

            if(!retreat_back.ok){
                throw new Error("Failed to execute script");
            }
        }
        
    } catch (error) {
        console.error('Error in Viewing Permission:', error);
    }
}


// ******************************************************************* Read NVM/CCB **********************************************************/

function NVMRead(){
    document.getElementById('read_tools_overlay_nvm').style.display = 'flex';
}

function closePopup(id) {
    document.getElementById(id).style.display = 'none';
}

function showPopUp_NVM(message) {
    const packetRegex = /Packet number : (\d+) \/ (\d+)/; 
    
 
    // Split the current terminal output into lines
    const outputLines = terminalOutput.textContent.split('\n');
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
    terminalOutput.textContent = outputLines.join('\n');
}

async function runNVMScript() {
    const fileName = document.getElementById('NVM_file-name').value;
    document.getElementById('read_tools_overlay_nvm').style.display = 'none';
    const path = 'Tools/host_boot_tools/NVM_Utils';
    const script = './nvm-read.sh';

    
    const response = await fetch('/run_shell_script', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ version: fileName,script: script})
    });

    if (!response.ok) {
        console.error('Error starting shell script:', response.statusText);
        return;
    }
    const eventSource = new EventSource('/run-script');
    let isEventSourceClosed = false;

    terminalOutput.textContent = ''; // Clear previous output
    document.getElementById('terminal_modal').style.display = 'block';
    terminalOutput.textContent = 'Reading NVM ...\n\n'; 

    eventSource.onmessage = function(event) {
        showPopUp_NVM(event.data);
    };

    eventSource.onerror = function(event) {
    eventSource.close();
    if(eventSource.readyState === 2) {
        console.log('Event source closed.')
        isEventSourceClosed = true;
        checkAndDownload();
    };
    };

    async function checkAndDownload() {
        if (isEventSourceClosed) {
            await download_button(path, fileName);
        }
    }
}

async function download_button(path,filename){
    try {
        const response = await fetch('/download', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ path, filename })
        });

        if (response.ok) {
            // Create and append the download button
            const downloadButton = document.createElement('button');
            downloadButton.innerText = 'Download';
            downloadButton.onclick = async () => {
                const blob = await response.blob();
                const url = window.URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = filename;
                document.body.appendChild(a);
                a.click();
                a.remove();
                window.URL.revokeObjectURL(url);
            };
            document.getElementById('terminal_output').appendChild(downloadButton);
        } else {
            const errorData = await response.json();
            console.error('Download failed:', errorData.error);
        }
    } catch (error) {
        console.error('Error checking file:', error);
    }
}

function CCBRead(){
    document.getElementById('read_tools_overlay_ccb').style.display = 'flex';
}

async function runCCBScript() {
    const fileName = document.getElementById('CCB_file-name').value;
    document.getElementById('read_tools_overlay_ccb').style.display = 'none';
    const path = 'Tools/host_boot_tools/NVM_Utils';
    const script = './ccb-read.sh';

    
    const response = await fetch('/run_shell_script', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ version: fileName,script: script})
    });

    if (!response.ok) {
        console.error('Error starting shell script:', response.statusText);
        return;
    }
    const eventSource = new EventSource('/run-script');
    let isEventSourceClosed = false;

    terminalOutput.textContent = ''; // Clear previous output
    document.getElementById('terminal_modal').style.display = 'block';
    terminalOutput.textContent = 'Reading CCB ...\n\n'; 

    eventSource.onmessage = function(event) {
        showPopUp_NVM(event.data);
    };

    eventSource.onerror = function(event) {
    eventSource.close();
    if(eventSource.readyState === 2) {
        console.log('Event source closed.')
        isEventSourceClosed = true;
        checkAndDownload();
    };
    };

    async function checkAndDownload() {
        if (isEventSourceClosed) {
            await download_button(path, fileName);
        }
    }
}