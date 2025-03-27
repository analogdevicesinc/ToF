use std::fs::{self, File};
use std::io::{self, Read};
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use reqwest::blocking::Client;
use reqwest::header::{USER_AGENT, CACHE_CONTROL, ACCEPT};

use rand::{distributions::Alphanumeric, Rng};
use sha2::{Sha256, Digest};

use crate::types::{VersionEntry, Config};

const VERSION_URL: &str = "https://swdownloads.analog.com/cse/aditof/aware3d/sw_update/adcam-mipi.json";
const RETRY_LIMIT: u32 = 5;
const RETRY_DELAY_SECS: u64 = 5;

pub fn fetch_version_json(client: &Client, local_path: Option<&str>) -> anyhow::Result<String> {
    if let Some(path) = local_path {
        let json_path = Path::new(path).join("adcam-mipi.json");
        return Ok(fs::read_to_string(&json_path)?);
    }

    let mut rng = rand::thread_rng();
    let buster: String = (0..8).map(|_| rng.sample(Alphanumeric) as char).collect();
    let busted_url = format!("{}?nocache={}", VERSION_URL, buster);

    for attempt in 1..=RETRY_LIMIT {
        let response = client
            .get(&busted_url)
            .header(USER_AGENT, "Mozilla/5.0")
            .header(CACHE_CONTROL, "no-cache")
            .header(ACCEPT, "application/json")
            .send();

        match response {
            Ok(resp) => {
                if resp.status().is_success() {
                    let body = resp.text()?;
                    if !body.trim().is_empty() {
                        return Ok(body);
                    }
                } else {
                    println!("⚠️ HTTP error: {}", resp.status());
                }
            }
            Err(e) => println!("⚠️ Request failed: {}", e),
        }

        if attempt < RETRY_LIMIT {
            println!("⏳ Retrying in {} seconds...", RETRY_DELAY_SECS);
            sleep(Duration::from_secs(RETRY_DELAY_SECS));
        }
    }

    Err(anyhow::anyhow!("Failed to fetch version info after {} attempts.", RETRY_LIMIT))
}

pub fn download_file(client: &Client, release: &VersionEntry, config: &Config, local_path: Option<&str>) -> anyhow::Result<()> {
    let path = if let Some(folder) = local_path {
        Path::new(folder).join(&release.filename).to_string_lossy().to_string()
    } else {
        let base_url = VERSION_URL.trim_end_matches("/adcam-mipi.json");
        let mut rng = rand::thread_rng();
        let buster: String = (0..8).map(|_| rng.sample(Alphanumeric) as char).collect();
        let file_url = format!("{}/{}?nocache={}", base_url, release.filename, buster);
        let tmp_path = format!("/tmp/{}", release.filename);

        println!("⬇️  Downloading {} to {}", release.version, tmp_path);
        let mut download = client
            .get(&file_url)
            .header(USER_AGENT, "Mozilla/5.0")
            .header(CACHE_CONTROL, "no-cache")
            .send()?;

        if !download.status().is_success() {
            return Err(anyhow::anyhow!("Download failed: HTTP {}", download.status()));
        }

        let mut out = File::create(&tmp_path)?;
        io::copy(&mut download, &mut out)?;
        tmp_path
    };

    // SHA256 verification
    let mut file = File::open(&path)?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 4096];
    while let Ok(n) = file.read(&mut buffer) {
        if n == 0 { break; }
        hasher.update(&buffer[..n]);
    }
    let calculated = format!("{:x}", hasher.finalize());
    if calculated != release.sha256.to_lowercase() {
        return Err(anyhow::anyhow!("SHA256 mismatch!"));
    }

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(&path, fs::Permissions::from_mode(0o755))?;
    }

    println!("✅ SHA256 OK. Running installer...");

    let status = std::process::Command::new(&path).status()?;
    if !status.success() {
        println!("⚠️ Installer exited with error.");
    }

    let dst_dir = format!("{}/installers", config.install_path_prefix.trim_end_matches('/'));
    fs::create_dir_all(&dst_dir)?;
    let dst_path = format!("{}/{}", dst_dir, release.filename);
    fs::copy(&path, &dst_path)?;
    println!("📦 Copied to: {}", dst_path);

    Ok(())
}
