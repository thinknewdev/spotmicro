#!/bin/bash

# Exit on any error
set -e

# Variables
VERSION="v1.8.0"  # Check https://github.com/bluenviron/mediamtx/releases for the latest
ARCH="arm64v8"    # Use "armv7" for 32-bit OS
DOWNLOAD_URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/mediamtx_${VERSION}_linux_${ARCH}.tar.gz"
INSTALL_DIR="/opt/mediamtx"

# Step 1: Update system
echo "Updating system..."
sudo apt update && sudo apt upgrade -y

# Step 2: Download MediaMTX
echo "Downloading MediaMTX ${VERSION} for ${ARCH}..."
wget -O mediamtx.tar.gz "$DOWNLOAD_URL"

# Step 3: Extract files
echo "Extracting files..."
tar -xvf mediamtx.tar.gz

# Step 4: Move to install directory
echo "Setting up installation directory..."
sudo mkdir -p "$INSTALL_DIR"
sudo mv mediamtx mediamtx.yml "$INSTALL_DIR/"

# Step 5: Configure basic mediamtx.yml
echo "Configuring mediamtx.yml with a test stream..."
sudo bash -c "cat > $INSTALL_DIR/mediamtx.yml" << EOL
paths:
  test:
    source: "testsrc"
EOL

# Step 6: Create systemd service
echo "Setting up MediaMTX as a service..."
sudo bash -c "cat > /etc/systemd/system/mediamtx.service" << EOL
[Unit]
Description=MediaMTX Service
Wants=network.target
After=network.target

[Service]
ExecStart=$INSTALL_DIR/mediamtx $INSTALL_DIR/mediamtx.yml
Restart=always

[Install]
WantedBy=multi-user.target
EOL

# Step 7: Enable and start the service
echo "Enabling and starting MediaMTX service..."
sudo systemctl daemon-reload
sudo systemctl enable mediamtx
sudo systemctl start mediamtx

# Cleanup
echo "Cleaning up..."
rm mediamtx.tar.gz

# Verify
echo "Checking service status..."
sudo systemctl status mediamtx --no-pager

echo "Installation complete! Test the stream with: rtsp://$(hostname -I | awk '{print $1}'):8554/test"
echo "Use VLC or another RTSP client to view it."
