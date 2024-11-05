# Dockerfile

# Use Ubuntu as the base image
FROM ubuntu:latest

# Install Python and necessary dependencies
RUN apt-get update && \
    apt-get install -y python3 python3-pip python3-venv && \
    apt-get clean

# Create a directory for venv and install requirements
WORKDIR /opt/app
COPY requirements.txt /opt/app/requirements.txt
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install -r /opt/app/requirements.txt

# Set /app as the working directory and add /opt/venv/bin to PATH
WORKDIR /app
ENV PATH="/opt/venv/bin:$PATH"

# Default command
CMD ["/opt/venv/bin/python3", "-m", "unittest", "discover", "-s", "tests"]
