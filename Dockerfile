FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
# uodate and install dependencies 
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# make and install the project
COPY cpp_bridge /cpp_bridge
WORKDIR /cpp_bridge
RUN ./install.sh && mkdir build && cd build && cmake .. && make

# set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
