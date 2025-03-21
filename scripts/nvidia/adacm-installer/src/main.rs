use include_dir::{include_dir, Dir};
use indicatif::{ProgressBar, ProgressStyle};
use serde::Deserialize;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

// Embed the resources folder
static RESOURCES: Dir = include_dir!("$CARGO_MANIFEST_DIR/resources");

// Configuration structure loaded from config.json
#[derive(Debug, Deserialize)]
struct Config {
    version: String,
    install_path_prefix: String,
}

fn main() {
    // Load and display license
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

    // Load config.json
    let config_file = RESOURCES
        .get_file("config.json")
        .expect("Missing config.json");
    let config_data = config_file
        .contents_utf8()
        .expect("config.json not UTF-8");
    let config: Config =
        serde_json::from_str(config_data).expect("Failed to parse config.json");

    // Build install path from prefix + version
    let expanded_prefix = shellexpand::tilde(&config.install_path_prefix).to_string();
    let install_dir = PathBuf::from(expanded_prefix).join(&config.version);

    println!("Installing to {:?}\n", install_dir);

    // Extract files with progress bar
    extract_dir_with_progress(&RESOURCES, &install_dir, "config.json").expect("Installation failed.");

    println!("\nInstallation complete.");
}

/// Recursively collect files (excluding one) and extract them with a progress bar
fn extract_dir_with_progress(dir: &Dir, target: &PathBuf, skip_filename: &str) -> std::io::Result<()> {
    let mut files = Vec::new();
    collect_files_recursively(dir, &mut files, skip_filename);

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
        pb.inc(1);
    }

    pb.finish_with_message("done");
    Ok(())
}

/// Helper to recursively collect embedded files, excluding `skip_filename`
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
