# Workspace Package Generator

This document outlines the procedure to create a compressed `.tar.gz` update file that bundles together essential components of a software system. The package includes the following directories:

- `ToF/`
- `Tools/`
- `module/`
- `services/`

These directories are packaged into a `.tar.gz` file, which can be uploaded as a new workspace using a PowerShell script.

---

## Directory Structure

### `ToF/`
Contains the ToF repository **without** Git metadata.

---

### `Tools/`
Includes utility scripts used for flashing firmware, capturing frames, and other system interactions.

You can copy it from the following GitHub path:

https://github.com/analogdevicesinc/ToF/tree/main/sdcard-images-utils/nxp/patches/ubuntu_overlay/step3/home/analog/Workspace

**Note:** Ensure all shell scripts have execution permissions using:
 ```bash
 chmod +x *.sh
 ```
---

### `module/`
Contains kernel driver files, typically with `.ko` extension.

**Example:**
- `adsd3500.ko`

---

### `services/`
Includes system services that run in the background and manage various aspects of the ToF system.

You can copy it from the following GitHub path:

https://github.com/analogdevicesinc/ToF/tree/main/sdcard-images-utils/nxp/patches/ubuntu_overlay/step1/usr/lib/systemd/system


**Typical contents:**
- `adi-backup.service`
- `adi-tof.service`
- `network-gadget.service`
- `uvc-gadget.service`

These services are typically managed by `systemd` and are responsible for launching and maintaining runtime operations.

---

## How to Create the `.tar.gz` Update File

Run the following command in the root directory containing the folders:

```bash
tar -czvf Filename.tar.gz FolderName
```
Replace Filename.tar.gz with your desired archive name and FolderName with the directory containing ToF, Tools, module, and services.

## How to Create the `.sha256` File

To ensure integrity, generate a SHA-256 checksum file for the .tar.gz archive.

If your archive is named Filename.tar.gz, run:

```
sha256sum Filename.tar.gz > Filename.tar.gz.sha256
```
This will create a file named Filename.tar.gz.sha256 containing the checksum, located in the same directory as the archive.
