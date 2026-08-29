# Base image with CUDA and Python support
ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}

# Set noninteractive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV UV_SYSTEM_PYTHON=1

# INSTALL UV
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Install System Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    libgl1 \
    libglib2.0-0 \
    python3-wxgtk4.0 \
    && rm -rf /var/lib/apt/lists/*

# INSTALL PROJECTAIRSIM
WORKDIR /tmp
RUN git clone --filter=blob:none --no-checkout https://github.com/iamaisim/ProjectAirSim.git \
    && cd ProjectAirSim \
    && git sparse-checkout init --cone \
    && git sparse-checkout set client/python/projectairsim \
    && git checkout 3302010393ac896e8dffc16cbbe2ec1e05d844e3 \
    && mv client/python/projectairsim /opt/projectairsim \
    && cd / \
    && rm -rf /tmp/ProjectAirSim

# INSTALL DEPENDENCIES
# Editable, so the pinned source stays readable and patchable inside the container.
# /opt/projectairsim must therefore survive in the image -- it is where imports resolve.
RUN uv pip install --system -e /opt/projectairsim

RUN uv pip install --system pre-commit

# Links the published package to this repo on GitHub
LABEL org.opencontainers.image.source=https://github.com/MissouriMRR/Simulation-2026
