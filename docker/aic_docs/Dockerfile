# Base image with Python
FROM python:3.12-slim

# Metadata
LABEL maintainer="Trushant Adeshara <trushant@intrinsic.ai>"

# Set working directory
WORKDIR /src

# Install OS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements or install manually
COPY docker/aic_docs/requirements.txt ./requirements.txt

RUN pip install --no-cache-dir -r requirements.txt

# Expose port for live preview
EXPOSE 8000

# Default command to run sphinx-autobuild
CMD ["sphinx-autobuild", "docs", "_build/html", "--host", "0.0.0.0", "--port", "8000"]
