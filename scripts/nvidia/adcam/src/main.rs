use std::fs::{self, File};
use std::io::{self, Write};
use std::collections::HashSet;

use reqwest::blocking::Client;
use reqwest::header::{USER_AGENT, CACHE_CONTROL, ACCEPT};
use serde::Deserialize;
use serde_json::from_str;

const VERSION_URL: &str = "https://swdownloads.analog.com/cse/aditof/aware3d/sw_update/adcam-mipi.json";
const REGISTRY_FILE: &str = ".registry.json";

#[derive(Debug, Deserialize)]
struct RegistryEntry {
    version: String,
    path: String,
}

#[derive(Debug, Deserialize)]
struct RemoteVersion {
    version: String,
    filename: String,
}

fn main() -> anyhow::Result<()> {
    println!("ADCAM Version Checker v{}", env!("CARGO_PKG_VERSION"));

    // Load local registry
    let registry_contents = fs::read_to_string(REGISTRY_FILE)
        .map_err(|e| anyhow::anyhow!("Failed to read {}: {}", REGISTRY_FILE, e))?;
    let registry: Vec<RegistryEntry> = from_str(&registry_contents)?;
    let installed_versions: HashSet<String> = registry
        .into_iter()
        .map(|entry| entry.version)
        .collect();

    // Fetch remote version.json
    let client = Client::new();
    let response = client
        .get(VERSION_URL)
        .header(USER_AGENT, "Mozilla/5.0 (compatible; VersionCheckBot/1.0)")
        .header(CACHE_CONTROL, "no-cache")
        .header(ACCEPT, "application/json")
        .send()?;

    let raw_text = response.text()?;
    if raw_text.trim().is_empty() {
        return Err(anyhow::anyhow!("Received empty response from server."));
    }

    let remote: RemoteVersion = from_str(&raw_text)?;

    // Check version
    if installed_versions.contains(&remote.version) {
        println!("✅ You already have the latest version: v{}", remote.version);
        return Ok(());
    }

    println!("🚨 New version available: v{}", remote.version);
    println!("Filename: {}", remote.filename);

    // Prompt user
    print!("Do you want to download this version? (y/n): ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let input = input.trim().to_lowercase();

    if input == "y" || input == "yes" {
        let base_url = VERSION_URL.trim_end_matches("/adcam-mipi.json");
        let file_url = format!("{}/{}", base_url, remote.filename);

        let mut download = client
            .get(&file_url)
            .header(USER_AGENT, "Mozilla/5.0 (compatible; VersionCheckBot/1.0)")
            .header(CACHE_CONTROL, "no-cache")
            .send()?;

        if !download.status().is_success() {
            return Err(anyhow::anyhow!("Download failed: HTTP {}", download.status()));
        }

        let mut out = File::create(&remote.filename)?;
        io::copy(&mut download, &mut out)?;
        println!("✅ Downloaded: {}", remote.filename);
    } else {
        println!("❌ Download canceled.");
    }

    Ok(())
}
