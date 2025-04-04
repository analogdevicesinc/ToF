# List of Debian packages to check and install

INTERNET_CONNECTION=false
host="8.8.8.8"
count=1
timeout=2

if ping -c "$count" -W "$timeout" "$host" > /dev/null 2>&1; then
  INTERNET_CONNECTION=true
else
  INTERNET_CONNECTION=false
fi

packages=(
  "cmake"
  "gcc"
  "g++"
  "libopencv-contrib-dev"
  "libopencv-dev"
  "libgl1-mesa-dev"
  "libglfw3-dev"
  "doxygen"
  "graphviz"
  "python3.10-dev"
  # Add more packages to this list
)

# Function to check if a package is installed
is_package_installed() {
  dpkg -s "$1" > /dev/null 2>&1
  return $? # Return 0 if installed, non-zero otherwise
}

# Loop through the list of packages
for package in "${packages[@]}"; do
  #echo "Checking if '$package' is installed..."
  if ! is_package_installed "$package"; then
    echo "Package '$package' is not installed. Attempting to install..."
    if [ "$INTERNET_CONNECTION" = false ]; then
      echo "❗No internet connection. Please connect to the internet and try again❗"
      echo "Continuing with remiander of script, but the features may be limited..."
      break
    fi
    # Install the package without prompting (-y or --force-yes)
    sudo apt-get update > /dev/null 2>&1
    sudo apt-get install -y "$package" > /dev/null 2>&1
    if [ $? -eq 0 ]; then
      echo "Successfully installed '$package'."
    else
      echo "Error installing '$package'. Please check for potential issues."
    fi
  #else
    #echo "Package '$package' is already installed."
  fi
done

echo "Package check and installation complete."