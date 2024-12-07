# Alternative setup script for Client UI (if you use homebrew)
brew --version > /dev/null 2>&1 || {
    echo "Must have homebrew installed from https://brew.sh to use this script"
    exit 1
}

brew install --HEAD numpy # source install to solve "not found import numpy.core.multiarray"
brew link --overwrite numpy
brew install pyqt@5 pillow opencv
