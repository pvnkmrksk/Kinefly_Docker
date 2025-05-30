#!/bin/bash
# Setup script for Kinefly ZMQ Bridge dependencies

set -e  # Exit on any error

echo "🚀 Setting up Kinefly ZMQ Bridge environment..."

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo "📦 Installing uv package manager..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    
    # Source the shell configuration to make uv available
    export PATH="$HOME/.local/bin:$PATH"
    
    echo "✅ uv installed successfully!"
else
    echo "✅ uv is already installed"
fi

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "🐍 Creating virtual environment..."
    uv venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# Install dependencies
echo "📚 Installing Python dependencies..."
uv pip install -r requirements.txt

echo "🎉 Setup complete!"
echo ""
echo "To activate the virtual environment, run:"
echo "source .venv/bin/activate"
echo ""
echo "Then you can run the bridge scripts:"
echo "python2 ros_zmq_bridge.py"
echo "python3 socket_zmq_republisher.py" 