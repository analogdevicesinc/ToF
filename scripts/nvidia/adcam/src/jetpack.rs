use regex::Regex;

pub fn detect_jetpack_version() -> Option<String> {
    let contents = std::fs::read_to_string("/etc/nv_tegra_release").ok()?;
    let release_re = Regex::new(r"R(\d+)").ok()?;
    let revision_re = Regex::new(r"REVISION:\s*(\d+)\.(\d+)").ok()?;

    let major = release_re.captures(&contents)?.get(1)?.as_str().parse::<u32>().ok()?;
    let (minor, _) = {
        let caps = revision_re.captures(&contents)?;
        (
            caps.get(1)?.as_str().parse::<u32>().ok()?,
            caps.get(2)?.as_str().parse::<u32>().ok()?,
        )
    };

    match (major, minor) {
        (36, 3..) => Some("6.2".to_string()),
        (36, 2) => Some("6.1".to_string()),
        (36, 1) => Some("6.0".to_string()),
        _ => None,
    }
}
