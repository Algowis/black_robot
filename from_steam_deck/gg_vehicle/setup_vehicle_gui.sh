#!/bin/bash

# Exit immediately if a command fails
set -e

# Define variables
VENV_NAME="venv"
PYTHON_SCRIPT="vehicle_gui.py"

echo "=== Steam Deck G.G. Setup ==="

# 1. Check if your Python script is actually in this folder
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Cannot find '$PYTHON_SCRIPT'."
    echo "Make sure this .sh script is in the same folder as your Python file."
    exit 1
fi

# 2. Create the virtual environment if it doesn't already exist
if [ ! -d "$VENV_NAME" ]; then
    echo "Creating virtual environment: $VENV_NAME..."
    python3 -m venv "$VENV_NAME"
else
    echo "Virtual environment '$VENV_NAME' already exists. Skipping creation."
fi

# 3. Activate the virtual environment
echo "Activating virtual environment..."
source "$VENV_NAME/bin/activate"

# 4. Update pip and install PyQt5
# (Checking if it's already installed speeds up subsequent runs)
if ! python -c "import PyQt5" &> /dev/null; then
    echo "PyQt5 not found. Checking internet connection..."

    # Wait until internet is available (ping Google DNS)
    until ping -c 1 -W 2 8.8.8.8 &> /dev/null; do
        echo "No internet connection. Waiting 5 seconds before retrying..."
        sleep 5
    done

    echo "Internet connection detected."
    echo "Upgrading pip..."
    pip install --upgrade pip --quiet
    echo "Installing PyQt5..."
    pip install PyQt5
else
    echo "PyQt5 is already installed."
fi

# 4. Update pip and install numpy
# (Checking if it's already installed speeds up subsequent runs)
if ! python -c "import numpy" &> /dev/null; then
    echo "numpy not found. Checking internet connection..."

    # Wait until internet is available (ping Google DNS)
    until ping -c 1 -W 2 8.8.8.8 &> /dev/null; do
        echo "No internet connection. Waiting 5 seconds before retrying..."
        sleep 5
    done

    echo "Internet connection detected."
    echo "Upgrading pip..."
    pip install --upgrade pip --quiet
    echo "Installing numpy..."
    pip install numpy
else
    echo "numpy is already installed."
fi

# 4. Update pip and install pygame
# (Checking if it's already installed speeds up subsequent runs)
if ! python -c "import pygame" &> /dev/null; then
    echo "pygame not found. Checking internet connection..."

    # Wait until internet is available (ping Google DNS)
    until ping -c 1 -W 2 8.8.8.8 &> /dev/null; do
        echo "No internet connection. Waiting 5 seconds before retrying..."
        sleep 5
    done

    echo "Internet connection detected."
    echo "Upgrading pip..."
    pip install --upgrade pip --quiet
    echo "Installing pygame..."
    pip install pygame
else
    echo "pygame is already installed."
fi

# 4. Update pip and install cv2
# (Checking if it's already installed speeds up subsequent runs)
if ! python -c "import cv2" &> /dev/null; then
    echo "cv2 not found. Checking internet connection..."

    # Wait until internet is available (ping Google DNS)
    until ping -c 1 -W 2 8.8.8.8 &> /dev/null; do
        echo "No internet connection. Waiting 5 seconds before retrying..."
        sleep 5
    done

    echo "Internet connection detected."
    echo "Upgrading pip..."
    pip install --upgrade pip --quiet
    echo "Installing cv2..."
    pip install opencv-python
else
    echo "cv2 is already installed."
fi

# 5. Run your custom GUI script
echo "Launching $PYTHON_SCRIPT..."
python "$PYTHON_SCRIPT"

# 6. Clean exit
echo "Application closed. Deactivating environment..."
deactivate
