use include_dir::{include_dir, Dir};
use indicatif::{ProgressBar, ProgressStyle};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

#[cfg(unix)]
use std::os::unix::fs::PermissionsExt;

static RESOURCES: Dir = include_dir!("$CARGO_MANIFEST_DIR/resources");
static CONFIGJSON: Dir = include_dir!("$CARGO_MANIFEST_DIR/..");

#[derive(Debug, Deserialize)]
struct Config {
    version: String,
    install_path_prefix: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct RegistryEntry {
    version: String,
    path: String,
}

fn main() {
    // Print tool version from Cargo.toml
    println!("ADCAM Installer v{}\n", env!("CARGO_PKG_VERSION"));

    // Load config.json FIRST
    let config_file = CONFIGJSON
        .get_file("config.json")
        .expect("Missing config.json");
    let config_data = config_file
        .contents_utf8()
        .expect("config.json not UTF-8");
    let config: Config =
        serde_json::from_str(config_data).expect("Failed to parse config.json");

    let expanded_prefix = shellexpand::tilde(&config.install_path_prefix).to_string();
    let install_dir = PathBuf::from(&expanded_prefix).join(&config.version);

    // Check registry before anything else
    if let Some(existing) = is_version_installed(&config.version, &config.install_path_prefix) {
        println!(
            "⚠️  Version {} is already installed at '{}'.",
            existing.version, existing.path
        );
        match prompt_user_choice() {
            1 => {
                println!("Proceeding to overwrite...\n");
            }
            2 => {
                match uninstall_version(&config.version, &config.install_path_prefix) {
                    Ok(_) => println!("✅ Uninstalled version {} successfully.", config.version),
                    Err(e) => println!("❌ Failed to uninstall: {}", e),
                }
                return;
            }
            _ => {
                println!("Installation aborted.");
                return;
            }
        }
    }

    // License agreement only shown AFTER check
    let license_file = RESOURCES
        .get_file("LICENSE.txt")
        .expect("LICENSE.txt not found in resources");
    let license = license_file
        .contents_utf8()
        .expect("LICENSE.txt is not valid UTF-8");

    println!("{}", license);
    print!("Do you accept the license agreement? (yes/no): ");
    io::stdout().flush().unwrap();

    let mut answer = String::new();
    io::stdin().read_line(&mut answer).unwrap();

    if answer.trim().to_lowercase() != "yes" {
        println!("License not accepted. Exiting.");
        return;
    }

    // Proceed with install
    println!("Installing to {:?}\n", install_dir);
    extract_dir_with_progress(&RESOURCES, &install_dir, "config.json").expect("Installation failed.");

    update_registry(
        &config.version,
        &install_dir.to_string_lossy(),
        &config.install_path_prefix,
    )
    .expect("Failed to update registry");

    println!("\nInstallation complete. Registry updated.");
}

fn prompt_user_choice() -> u8 {
    loop {
        println!("\nWhat would you like to do?");
        println!("[1] Overwrite");
        println!("[2] Uninstall");
        println!("[3] Quit");

        print!("Enter your choice [1-3]: ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();

        match input.trim() {
            "1" => return 1,
            "2" => return 2,
            "3" => return 3,
            _ => {
                println!("Invalid choice. Please enter 1, 2, or 3.");
            }
        }
    }
}

fn uninstall_version(version: &str, install_path_prefix: &str) -> std::io::Result<()> {
    let expanded_prefix = shellexpand::tilde(install_path_prefix).to_string();
    let registry_path = PathBuf::from(&expanded_prefix).join(".registry.json");
    let version_path = PathBuf::from(&expanded_prefix).join(version);

    if version_path.exists() {
        fs::remove_dir_all(&version_path)?;
    }

    let mut registry: Vec<RegistryEntry> = Vec::new();
    if registry_path.exists() {
        let data = fs::read_to_string(&registry_path)?;
        registry = serde_json::from_str(&data).unwrap_or_else(|_| Vec::new());
    }

    registry.retain(|entry| entry.version != version);

    let json = serde_json::to_string_pretty(&registry)?;
    fs::write(&registry_path, json)?;

    Ok(())
}

fn is_version_installed(version: &str, install_path_prefix: &str) -> Option<RegistryEntry> {
    let expanded_prefix = shellexpand::tilde(install_path_prefix).to_string();
    let registry_path = PathBuf::from(expanded_prefix).join(".registry.json");

    if registry_path.exists() {
        if let Ok(data) = fs::read_to_string(&registry_path) {
            if let Ok(parsed) = serde_json::from_str::<Vec<RegistryEntry>>(&data) {
                return parsed.into_iter().find(|entry| entry.version == version);
            }
        }
    }

    None
}

fn update_registry(version: &str, path: &str, install_path_prefix: &str) -> std::io::Result<()> {
    let expanded_prefix = shellexpand::tilde(install_path_prefix).to_string();
    let registry_path = PathBuf::from(expanded_prefix).join(".registry.json");

    if let Some(parent) = registry_path.parent() {
        fs::create_dir_all(parent)?;
    }

    let mut registry: Vec<RegistryEntry> = Vec::new();

    if registry_path.exists() {
        let data = fs::read_to_string(&registry_path)?;
        registry = serde_json::from_str(&data).unwrap_or_else(|_| Vec::new());
    }

    registry.retain(|entry| entry.version != version);

    registry.push(RegistryEntry {
        version: version.to_string(),
        path: path.to_string(),
    });

    let json = serde_json::to_string_pretty(&registry)?;
    fs::write(&registry_path, json)?;

    Ok(())
}

fn extract_dir_with_progress(dir: &Dir, target: &PathBuf, skip_filename: &str) -> std::io::Result<()> {
    let mut files = Vec::new();
    collect_files_recursively(dir, &mut files, skip_filename);

    let permissions_map = load_permissions_map();

    let pb = ProgressBar::new(files.len() as u64);
    pb.set_style(
        ProgressStyle::with_template("[{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} {msg}")
            .unwrap()
            .progress_chars("##-"),
    );

    for (relative_path, contents) in files {
        let target_path = target.join(&relative_path);
        if let Some(parent) = target_path.parent() {
            fs::create_dir_all(parent)?;
        }

        fs::write(&target_path, contents)?;

        if let Some(mode) = permissions_map.get(&relative_path.to_string_lossy().to_string()) {
            #[cfg(unix)]
            fs::set_permissions(&target_path, fs::Permissions::from_mode(*mode))?;
        }

        pb.inc(1);
    }

    pb.finish_with_message("done");
    Ok(())
}

fn collect_files_recursively<'a>(
    dir: &'a Dir<'a>,
    files: &mut Vec<(PathBuf, &'a [u8])>,
    skip_filename: &str,
) {
    for file in dir.files() {
        if file.path().file_name().map(|n| n == skip_filename).unwrap_or(false) {
            continue;
        }
        files.push((file.path().to_path_buf(), file.contents()));
    }

    for sub in dir.dirs() {
        collect_files_recursively(sub, files, skip_filename);
    }
}

fn load_permissions_map() -> HashMap<String, u32> {
    let permissions_file = RESOURCES
        .get_file("permissions.json")
        .expect("Missing permissions.json");
    let json = permissions_file
        .contents_utf8()
        .expect("permissions.json is not valid UTF-8");
    serde_json::from_str(json).expect("Failed to parse permissions.json")
}
