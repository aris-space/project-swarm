# Base image
FROM arm64v8/python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    gfortran \
    python3-dev \
    libatlas-base-dev \
    libopenblas-dev \
    liblapack-dev \
    libfreetype6-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip install --upgrade pip setuptools wheel

# Install Python dependencies
RUN pip install --no-cache-dir \
    numpy \
    pyyaml \
    scipy \
    pyserial \
    adafruit-bno055 \
    pigpio \
    matplotlib \
    adafruit-circuitpython-bno055

# build this Docker image by (note, scond is name): docker build linux/arm/v7 -t my-conda-python-3.11 
# need this for 32 bit controllers: docker build --platform linux/arm/v7 -t my-python-image .
# spin up container and land in bash docker run -it python-3.11 /bin/bash

