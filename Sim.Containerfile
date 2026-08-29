FROM docker.io/ardupilot/ardupilot-dev-base

# INSTALL DEPENDENCIES

RUN apt install -y software-properties-common && apt update && add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y tmux iproute2
RUN apt-get install python3-wxgtk4.0 -y --no-install-recommends

# ARDUPILOT

RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot

WORKDIR ardupilot

ARG ARDU_BRANCH=Copter-4.5
RUN git checkout $ARDU_BRANCH

RUN git submodule update --init --recursive

RUN ./waf configure --board sitl && ./waf copter

RUN pip install mavproxy geocoder matplotlib numpy opencv-python
RUN pip install -U numpy

# add GolfCourse location to ArduPilot locations
RUN echo '# Multirotor Locations\nGolfCourse=37.9490953,-91.7848293,0,0' >> /ardupilot/Tools/autotest/locations.txt

COPY ./sim_start_drones.sh /ardupilot/Tools/autotest/
COPY ./templates/multidrone.parm /ardupilot/Tools/autotest/

# Strip carriage returns in case the build context was checked out on Windows with CRLF
# endings. .gitattributes should already prevent this
RUN sed -i 's/\r$//' /ardupilot/Tools/autotest/sim_start_drones.sh \
    && chmod +x /ardupilot/Tools/autotest/sim_start_drones.sh

LABEL org.opencontainers.image.source=https://github.com/MissouriMRR/Simulation-2026

# ENVIRONMENT VARIABLES

# The variables below are passed to sim_start_drones.sh within the container (see below)
# NOTE: DO NOT CHANGE THESE HERE (unless defaults change); set the env variables when
#       starting the sim container (including when using run_container.sh)

# OUT_PORT/HOST - sets the port/IP of the sim
ENV OUT_PORT=14550
ENV OUT_HOST=127.0.0.1

# NUM_DRONES - the number of drones to start
#  for multi-drone simulations, it's recommended to use update_airsim_settings.ps1 to automatically
#     configure settings correctly.
ENV NUM_DRONES=1

CMD /ardupilot/Tools/autotest/sim_start_drones.sh $NUM_DRONES $OUT_PORT $OUT_HOST