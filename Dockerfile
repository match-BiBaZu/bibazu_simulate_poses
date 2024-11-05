# Use Ubuntu as the base image
FROM ubuntu:latest  
# or specify a version, e.g., ubuntu:24.04

# Install Python and other necessary dependencies
RUN apt-get update && \
    apt-get install -y python3 python3-pip python3-venv python3-dev build-essential && \
    apt-get clean

# Set the working directory inside the container
WORKDIR /app

# Copy all files from the current directory on your host machine into /app in the container
COPY . /app

# Set Python 3 as the default python command
RUN python3 -m venv /app/venv && \
    /app/venv/bin/pip install --no-cache-dir -r requirements.txt

# Activate the virtual environment and set it as the default
ENV PATH="/app/venv/bin:$PATH"

# Optional: Run any additional setup commands, such as installing dependencies
# If you have a requirements.txt, you could install dependencies like this:
RUN pip install -r requirements.txt

# Default command to run when the container starts
CMD ["python", "Main.py"]  
