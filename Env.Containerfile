# Base image with CUDA and Python support
ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}

# Set noninteractive frontend for apt
ENV DEBIAN_FRONTEND=noninteractive

# Ensure pip is up to date
RUN python -m pip install --upgrade pip

# INSTALL POETRY

ENV POETRY_HOME=/etc/poetry \
    POETRY_VERSION=1.8.5

RUN curl -sSL https://install.python-poetry.org | python3 -
ENV PATH="$POETRY_HOME/bin:$PATH"

# INSTALL PROJECTAIRSIM

RUN mkdir /pyenv

RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

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

# install stuff to global python environment instead of creating a virtualenv
# the container is our virtual environment
ENV POETRY_VIRTUALENVS_CREATE=false

# this command takes approximately 10 years to run
RUN poetry install --no-interaction --no-ansi

RUN python -m pip install -e /pyenv/projectairsim

# essential dependencies
RUN apt-get update && apt-get install -y libgl1 libglib2.0-0 python3-wxgtk4.0 git curl

RUN pip install pre-commit