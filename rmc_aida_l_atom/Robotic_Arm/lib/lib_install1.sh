#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: Your Name
# Description: Install libapi_cpp.so for target system

set -e

# Check sudo privileges
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo. Example: sudo $0"
    exit 1
fi

# Cleanup old library
echo "Removing existing libapi_cpp.so..."
sudo rm -f /usr/local/lib/libapi_cpp.so* 2>/dev/null || true

# Install new library
echo "Copying libapi_cpp.so to /usr/local/lib..."
if [ -f "./libapi_cpp.so" ]; then
    sudo cp ./libapi_cpp.so /usr/local/lib/
else
    echo "Error: libapi_cpp.so not found in current directory!"
    exit 1
fi

# Configure library path
echo "Updating system library configuration..."
if ! grep -q "/usr/local/lib" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
fi

# Update library cache
sudo ldconfig

# Completion message
echo -e "\n\033[1;32m[SUCCESS] libapi_cpp.so installed successfully!\033[0m"