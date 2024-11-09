# Use Ubuntu as the base image
FROM ubuntu:latest

# Install Python and other necessary dependencies
RUN apt-get update && \
    apt-get install -y python3 python3-pip python3-venv python3-dev build-essential libx11-6 && \
    apt-get clean

# Set the working directory inside the container
WORKDIR /app

# Copy the requirements file from the host to the container
COPY requirements.txt /app/requirements.txt

# Create a virtual environment and install requirements
RUN python3 -m venv /app/venv && \
    /app/venv/bin/pip install --no-cache-dir -r /app/requirements.txt

# Activate the virtual environment and set it as the default
ENV PATH="/app/venv/bin:$PATH"

# Create a symbolic link that will be updated at runtime
RUN ln -s /external_repo /app/external_repo

# Default command to run when the container starts
CMD ["python", "/app/external_repo/Main.py"]
