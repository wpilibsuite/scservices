#!/usr/bin/env bash
set -euo pipefail

# Install aarch64 Linux compiler on macOS:
# 
# brew tap messense/macos-cross-toolchains && brew install aarch64-unknown-linux-gnu

log() {
    echo "[$(date)] $1"
}

SRC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

pushd "$SRC_ROOT"

log "Configuring via CMake (cmake --preset mrc)"
cmake --preset mrc

log "Building via CMake (cmake --build --preset mrc)"
cmake --build --preset mrc

log "Stopping any existing instance of ExpansionHubDaemon service on target"
ssh systemcore@robot.local "sudo systemctl stop expansionhubdaemon"

log "Copying ExpansionHubDaemon to target"
scp "$SRC_ROOT/buildmrc/bin/ExpansionHubDaemon" systemcore@robot.local:~/ExpansionHubDaemon

log "Installing ExpansionHubDaemon to /usr/bin/ on target"
ssh systemcore@robot.local "sudo mv ~/ExpansionHubDaemon /usr/bin/ExpansionHubDaemon && sudo chmod +x /usr/bin/ExpansionHubDaemon"

log "Starting expansionhubdaemon service on target"
ssh systemcore@robot.local "sudo systemctl start expansionhubdaemon"

popd
