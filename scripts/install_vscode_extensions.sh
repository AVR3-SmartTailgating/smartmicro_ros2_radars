#!/bin/bash
# Install recommended VS Code extensions for ROS2 development

echo "Installing VS Code extensions for ROS2 development..."
echo ""

# Essential ROS2 extensions
echo "📦 Installing ROS Extension (Microsoft)..."
code --install-extension ms-iot.vscode-ros

echo "📦 Installing Robot Developer Extensions for ROS 2..."
code --install-extension Ranch-Hand-Robotics.rde-ros-2 || echo "⚠️  Optional extension not found, skipping..."

# Language support
echo "📦 Installing C/C++ Extension Pack..."
code --install-extension ms-vscode.cpptools-extension-pack

echo "📦 Installing Python Extension..."
code --install-extension ms-python.python

# Build tools
echo "📦 Installing CMake Tools..."
code --install-extension ms-vscode.cmake-tools

# Configuration file support
echo "📦 Installing YAML Support..."
code --install-extension redhat.vscode-yaml

echo "📦 Installing XML Tools..."
code --install-extension dotjoshjohnson.xml

# Nice-to-have extensions
echo ""
echo "📦 Installing optional productivity extensions..."
code --install-extension eamodio.gitlens || echo "⚠️  GitLens not found, skipping..."
code --install-extension mhutchie.git-graph || echo "⚠️  Git Graph not found, skipping..."

echo ""
echo "✅ Installation complete!"
echo ""
echo "Recommended next steps:"
echo "  1. Restart VS Code"
echo "  2. Open your ROS2 workspace: code ~/tailgate_ws"
echo "  3. Configure the ROS extension (see workspace settings)"
