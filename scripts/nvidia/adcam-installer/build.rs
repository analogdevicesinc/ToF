use std::{
    env,
    fs::{self, File},
    io::Write,
    os::unix::fs::PermissionsExt,
    path::{Path, PathBuf},
};

use walkdir::WalkDir;
use serde_json::json;

fn main() {
    let resources_dir = PathBuf::from("resources");
    let permissions_path = resources_dir.join("permissions.json");

    let mut map = serde_json::Map::new();

    for entry in WalkDir::new(&resources_dir)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| e.file_type().is_file())
    {
        let path = entry.path();
        let rel_path = path.strip_prefix(&resources_dir).unwrap();
        let metadata = fs::metadata(path).unwrap();
        let mode = metadata.permissions().mode() & 0o777;

        map.insert(rel_path.to_string_lossy().to_string(), json!(mode));
    }

    let output = json!(map);
    let mut file = File::create(&permissions_path).expect("Unable to create permissions.json");
    write!(file, "{}", output.to_string()).expect("Unable to write permissions.json");

    println!("cargo:rerun-if-changed=resources/");
}
