mod config;
mod jetpack;
mod remote;
mod types;

use clap::{Parser, CommandFactory};
use reqwest::blocking::Client;
use serde_json::from_str;
use types::VersionList;

#[derive(Parser)]
#[command(
    name = "adcam",
    version = env!("CARGO_PKG_VERSION"),
    about = concat!(
        "ADCAM Installer CLI - Version ",
        env!("CARGO_PKG_VERSION")
    ),
    long_about = concat!(
        "Checks and installs the latest ADCAM software.\n\n",
        "Version: ", env!("CARGO_PKG_VERSION")
    )    
)]
struct Cli {
    #[arg(short = 'i', long = "install")]
    install: Option<String>,

    #[arg(short = 'u', long = "uninstall")]
    uninstall: Option<String>,

    #[arg(short = 'l', long = "local")]
    local: Option<String>,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    if std::env::args().len() == 1 {
        Cli::command().print_help()?;
        println!();
        return Ok(());
    }
    
    let config = config::load_config()?;
    let client = Client::new();

    let raw_json = remote::fetch_version_json(&client, cli.local.as_deref())?;
    let version_list: VersionList = from_str(&raw_json)?;

    if let Some(actual_jp) = jetpack::detect_jetpack_version() {
        if version_list.jet_pack != actual_jp {
            return Err(anyhow::anyhow!(
                "JetPack mismatch: expected {}, found {}",
                version_list.jet_pack, actual_jp
            ));
        }
    }

    if let Some(requested_version) = cli.install {
        let release = &version_list.installer;
        if release.version == requested_version {
            remote::download_file(&client, release, &config, cli.local.as_deref())?;
        } else {
            println!("Version {} not found.", requested_version);
        }
        return Ok(());
    }

    println!("✅ Ready. No action taken.");
    Ok(())
}
