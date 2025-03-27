use std::fs::{self, File};
use std::path::{Path, PathBuf};
use crate::types::Config;

pub fn load_config() -> anyhow::Result<Config> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let config_path = Path::new(manifest_dir).join("..").join("config.json");

    let config_str = fs::read_to_string(&config_path)?;
    let mut config: Config = serde_json::from_str(&config_str)?;

    let prefix = &config.install_path_prefix;
    let expanded_path = if prefix.starts_with("~/") {
        std::env::var("HOME")
            .map(|home| PathBuf::from(home).join(&prefix[2..]))
            .map_err(|_| anyhow::anyhow!("HOME environment variable not set"))?
    } else if prefix.starts_with("$HOME/") {
        std::env::var("HOME")
            .map(|home| PathBuf::from(home).join(&prefix[6..]))
            .map_err(|_| anyhow::anyhow!("HOME environment variable not set"))?
    } else {
        PathBuf::from(prefix)
    };

    config.install_path_prefix = expanded_path.to_string_lossy().to_string();
    fs::create_dir_all(&config.install_path_prefix)?;

    let test_path = Path::new(&config.install_path_prefix).join(".test_write");
    File::create(&test_path)
        .and_then(|_| fs::remove_file(&test_path))
        .map_err(|e| {
            anyhow::anyhow!(
                "install_path_prefix is not writable: {}\nCaused by: {}",
                config.install_path_prefix,
                e
            )
        })?;

    Ok(config)
}
