use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct RegistryEntry {
    pub version: String,
    pub path: String,
}

#[derive(Debug, Deserialize)]
pub struct VersionEntry {
    pub version: String,
    pub filename: String,
    #[serde(rename = "sa256hash")]
    pub sha256: String,
}

#[derive(Debug, Deserialize)]
pub struct VersionList {
    pub jet_pack: String,
    pub user: VersionEntry,
    pub kernel: VersionEntry,
    pub installer: VersionEntry,
}

#[derive(Debug, Deserialize)]
pub struct Config {
    pub version: String,
    pub install_path_prefix: String,
}
