use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::collections::HashSet;
use std::thread::sleep;
use std::time::Duration;
use std::path::PathBuf;

use reqwest::blocking::Client;
use reqwest::header::{USER_AGENT, CACHE_CONTROL, ACCEPT};
use serde::Deserialize;
use serde_json::from_str;

use sha2::{Digest, Sha256};

use clap::Parser;

const VERSION_URL: &str = "https://swdownloads.analog.com/cse/aditof/aware3d/sw_update/adcam-mipi.json";

const RETRY_LIMIT: u32 = 5;
const RETRY_DELAY_SECS: u64 = 5;

#[derive(Debug, Deserialize)]
struct RegistryEntry {
    version: String,
    path: String,
}

#[derive(Debug, Deserialize)]
struct Release {
    version: String,
    filename: String,
    #[serde(rename = "sa256hash")]
    sha256: String,
}

#[derive(Debug, Deserialize)]
struct VersionList {
    releases: Vec<Release>,
}

#[derive(Parser)]
#[command(name = "adcam", version = env!("CARGO_PKG_VERSION"))]
#[command(about = "Checks and downloads the latest ADCAM installer.", long_about = None)]
struct Cli {
    /// List all available versions from the remote JSON
    #[arg(short, long)]
    list: bool,

    /// Download a specific version (e.g. 1.0.0)
    #[arg(short, long)]
    download: Option<String>,

    /// Run the uninstaller for a specific version (e.g. 1.0.0)
    #[arg(short, long)]
    uninstall: Option<String>,
}

fn get_registry_path() -> PathBuf {
    let home = std::env::var("HOME").expect("HOME environment variable not set");
    PathBuf::from(format!(
        "{}/Analog Devices/Robotics/Camera/ADCAM/.registry.json",
        home
    ))
}

fn fetch_version_json(client: &Client) -> anyhow::Result<String> {
    for attempt in 1..=RETRY_LIMIT {
        let response = client
            .get(VERSION_URL)
            .header(USER_AGENT, "Mozilla/5.0 (compatible; ADCAM/1.0)")
            .header(CACHE_CONTROL, "no-cache")
            .header(ACCEPT, "application/json")
            .send();

        match response {
            Ok(resp) => {
                if !resp.status().is_success() {
                    println!("⚠️ HTTP error: {}", resp.status());
                } else {
                    let body = resp.text()?;
                    if body.trim().is_empty() {
                        println!("⚠️ Empty response body.");
                    } else {
                        return Ok(body);
                    }
                }
            }
            Err(e) => {
                println!("⚠️ Request failed: {}", e);
            }
        }

        if attempt < RETRY_LIMIT {
            println!("⏳ Retrying in {} seconds...", RETRY_DELAY_SECS);
            sleep(Duration::from_secs(RETRY_DELAY_SECS));
        }
    }

    Err(anyhow::anyhow!("Failed to fetch version info after {} attempts.", RETRY_LIMIT))
}

fn print_versions(version_list: &VersionList) {
    println!("Available versions:");
    for release in &version_list.releases {
        println!("- {} ({})", release.version, release.filename);
    }
}

fn download_file(client: &Client, release: &Release) -> anyhow::Result<()> {
    let base_url = VERSION_URL.trim_end_matches("/adcam-mipi.json");
    let file_url = format!("{}/{}", base_url, release.filename);
    let path = format!("/tmp/{}", release.filename);

    println!("⬇️  Downloading {} to {}", release.version, path);

    let mut download = client
        .get(&file_url)
        .header(USER_AGENT, "Mozilla/5.0 (compatible; ADCAM/1.0)")
        .header(CACHE_CONTROL, "no-cache")
        .send()?;

    if !download.status().is_success() {
        return Err(anyhow::anyhow!("Download failed: HTTP {}", download.status()));
    }

    let mut out = File::create(&path)?;
    io::copy(&mut download, &mut out)?;
    drop(out);
    std::thread::sleep(std::time::Duration::from_millis(100));

    println!("✅ Downloaded to: {}", path);

    // SHA256 hash verification
    let mut file = File::open(&path)?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 4096];
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        hasher.update(&buffer[..n]);
    }
    let calculated_hash = format!("{:x}", hasher.finalize());

    if calculated_hash != release.sha256.to_lowercase() {
        println!("❌ SHA256 hash mismatch!");
        println!("Expected: {}", release.sha256);
        println!("Found:    {}", calculated_hash);
        std::fs::remove_file(&path)?;
        return Err(anyhow::anyhow!("File hash verification failed."));
    }

    println!("✅ SHA256 hash verified.");

    // chmod +x
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let perms = fs::Permissions::from_mode(0o755);
        fs::set_permissions(&path, perms)?;
    }

    // Ask to install
    print!("Do you want to run the installer now? (y/n): ");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim().to_lowercase();

    if input == "y" || input == "yes" {
        println!("🚀 Launching installer...");
        let status = std::process::Command::new(&path).status()?;

        if status.success() {
            println!("✅ Installer exited successfully.");
        } else {
            println!("⚠️ Installer exited with error.");
        }
    } else {
        println!("ℹ️ Installer was not launched.");
    }

    let home = std::env::var("HOME").expect("HOME not set");
    let installers_dir = format!("{}/Analog Devices/Robotics/Camera/ADCAM/installers", home);
    let destination_path = format!("{}/{}", installers_dir, release.filename);

    fs::create_dir_all(&installers_dir)?; // ensure folder exists
    fs::copy(&path, &destination_path)?;  // overwrite if exists

    println!("📦 Copied installer to: {}", destination_path);

    Ok(())
}

fn main() -> anyhow::Result<()> {
    if std::env::args().len() == 1 {
        Cli::parse_from(&["adcam", "--help"]);
        return Ok(());
    }

    let cli = Cli::parse();

    // Handle uninstall first
    if let Some(version) = &cli.uninstall {
        let home = std::env::var("HOME").expect("HOME not set");
        let installer_path = format!(
            "{}/Analog Devices/Robotics/Camera/ADCAM/installers/adcam-installer-{}.bin",
            home, version
        );

        if std::path::Path::new(&installer_path).exists() {
            println!("🧹 Running uninstaller at: {}", installer_path);
            let status = std::process::Command::new(&installer_path).status()?;
            if status.success() {
                println!("✅ Uninstaller exited successfully.");
            } else {
                println!("⚠️ Uninstaller exited with error.");
            }
        } else {
            println!("❌ Installer not found at: {}", installer_path);
        }

        return Ok(());
    }

    let client = Client::new();
    let raw_text = fetch_version_json(&client)?;
    let version_list: VersionList = from_str(&raw_text)?;

    if cli.list {
        print_versions(&version_list);
        return Ok(());
    }

    if let Some(requested_version) = cli.download {
        match version_list.releases.iter().find(|r| r.version == requested_version) {
            Some(release) => download_file(&client, release)?,
            None => {
                println!("❌ Version {} not found in remote list.", requested_version);
            }
        }
        return Ok(());
    }

    // Default: check latest version and prompt to download
    let registry_path = get_registry_path();

    let installed_versions: HashSet<String> = if registry_path.exists() {
        let registry_contents = fs::read_to_string(&registry_path)?;
        let registry: Vec<RegistryEntry> = from_str(&registry_contents)?;
        registry.into_iter().map(|entry| entry.version).collect()
    } else {
        println!("ℹ️ No registry file found at {:?}. Assuming no versions are installed.", registry_path);
        HashSet::new()
    };

    let latest = version_list
        .releases
        .first()
        .ok_or_else(|| anyhow::anyhow!("No releases found in version list."))?;

    if installed_versions.contains(&latest.version) {
        println!("✅ You already have the latest version: v{}", latest.version);
        return Ok(());
    }

    println!("🚨 New version available: v{}", latest.version);
    println!("Filename: {}", latest.filename);

    print!("Do you want to download this version? (y/n): ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim().to_lowercase();

    if input == "y" || input == "yes" {
        download_file(&client, latest)?;
    } else {
        println!("❌ Download canceled.");
    }

    Ok(())
}
