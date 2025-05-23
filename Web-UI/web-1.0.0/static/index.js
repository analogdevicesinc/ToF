function showOverlay(message = "Processing, please wait...") {
  const overlay = document.getElementById("overlay");
  const overlayText = document.getElementById("overlay-text");
  overlayText.textContent = message;
  overlay.style.display = "flex";
}

function hideOverlay() {
  document.getElementById("overlay").style.display = "none";
}

function openTab(event, tabId) {
  // Hide all content sections
  const contents = document.querySelectorAll(".content");
  contents.forEach((content) => {
    content.style.display = "none";
    content.classList.remove("active");
  });

  // Remove active class from all tabs
  const tabs = document.querySelectorAll(".tab");
  tabs.forEach((tab) => {
    tab.classList.remove("active");
  });

  // Show the selected tab's content and set it as active
  document.getElementById(tabId).style.display = "block";
  document.getElementById(tabId).classList.add("active");
  event.currentTarget.classList.add("active");
}

async function runService() {
  const outputElement = document.getElementById("serviceOutput");
  outputElement.textContent = "Running service...";

  try {
    const response = await fetch("/run-service-backup-nvm", { method: "POST" });

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
    fetch("/check-file-sizes")
      .then((response) => response.json())
      .then((data) => {
        if (data.error) {
          console.error("Error checking file sizes:", data.error);
          hideOverlay(); // Hide overlay on error
          return;
        }

        const { ccb_size, nvm_size, is_complete } = data;
        console.log(`CCB File: ${ccb_size} bytes, NVM File: ${nvm_size} bytes`);

        // Update the UI to show file sizes
        document.getElementById("fileSizes").textContent =
          `CCB File: ${(ccb_size / 1024).toFixed(2)} KB, ` +
          `NVM File: ${(nvm_size / 1024).toFixed(2)} KB`;

        // Stop polling if both files meet the size threshold
        if (is_complete) {
          console.log("Both files have reached the size threshold. Stopping.");
          clearInterval(pollingId);
          hideOverlay(); // Hide overlay on error

          // Optional: Show a success message
          document.getElementById("statusMessage").textContent =
            "Both files have reached the required size!";
        }
      })
      .catch((error) => console.error("Error during polling:", error));
    hideOverlay(); // Hide overlay on error
  }

  // Start polling
  const pollingId = setInterval(checkFileSizes, pollingInterval);
  runService();
  checkFileSizes(); // Initial check
}

// ********************************************************** change the color on selecting the section **************************************************/
document.addEventListener("DOMContentLoaded", function () {
  const navLinks = document.querySelectorAll(".nav_link a");

  navLinks.forEach((link) => {
    link.addEventListener("click", function (event) {
      // Remove active class from all links
      navLinks.forEach((link) => link.parentElement.classList.remove("active"));

      // Add active class to the clicked link
      this.parentElement.classList.add("active");
    });
  });
});

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// ******************************************************************** show the latest release *****************************************************/

async function fetchLatestRelease() {
  const response = await fetch(
    "https://api.github.com/repos/analogdevicesinc/ToF/releases/latest"
  );
  const data = await response.json();
  return data;
}

async function showLatestRelease() {
  const versionElement = document.getElementById("latest-version");
  const downloadLinkElement = document.getElementById("download-link");
  const releaseDateElement = document.getElementById("release-date");
  try {
    const latestRelease = await fetchLatestRelease();
    const latestVersion = latestRelease.tag_name;
    const releaseDate = new Date(
      latestRelease.published_at
    ).toLocaleDateString(); // Extract and format release date

    // Determine the user's operating system
    const userAgent = navigator.userAgent;
    const isWindows = userAgent.indexOf("Win") > -1;
    const isLinux = userAgent.indexOf("Linux") > -1;

    // Find the appropriate asset for the user's OS
    let downloadUrl = "";
    latestRelease.assets.forEach((asset) => {
      if (isWindows && asset.name.includes(".exe")) {
        downloadUrl = asset.browser_download_url;
      } else if (isLinux && asset.name.includes(".sh")) {
        downloadUrl = asset.browser_download_url;
      }
    });

    versionElement.textContent = `Latest Release: ${latestVersion}`;
    downloadLinkElement.href = downloadUrl;
    downloadLinkElement.textContent = "Download";
    releaseDateElement.textContent = `Release Date: ${releaseDate}`; // Display release date
  } catch (error) {
    console.error("Error fetching latest release:", error);
    versionElement.textContent = "Error fetching latest release";
    downloadLinkElement.textContent = "";
    releaseDateElement.textContent = ""; // Clear release date on error
  }
}

document.addEventListener("DOMContentLoaded", showLatestRelease);


