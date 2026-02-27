# Base image with CUDA and Python support
ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}

# Set noninteractive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Ensure pip is up to date
RUN python -m pip install --upgrade pip

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
RUN mkdir /pyenv
WORKDIR /tmp
RUN git clone --filter=blob:none --no-checkout https://github.com/iamaisim/ProjectAirSim.git \
    && cd ProjectAirSim \
    && git sparse-checkout init --cone \
    && git sparse-checkout set client/python/projectairsim \
    && git checkout 3302010393ac896e8dffc16cbbe2ec1e05d844e3 \
    && mv client/python/projectairsim /pyenv/projectairsim \
    && cd / \
    && rm -rf /tmp/ProjectAirSim

# INSTALL DEPENDENCIES
WORKDIR /pyenv

COPY ./pyproject.toml ./

RUN uv export --format requirements-txt --output-file requirements.txt && \
    uv pip install --system -r requirements.txt

RUN python -m pip install -e /pyenv/projectairsim

RUN uv pip install --system pre-commit

RUN uv cache clean