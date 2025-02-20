FROM ubuntu:24.04
LABEL Name=unitreeleggedsdk Version=0.0.1

# # Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# # Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
# ARG USERNAME=ubuntu
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# RUN if getent passwd ubuntu > /dev/null 2>&1; then \
#         userdel -r ubuntu && \
#         echo "Deleted existing ubuntu user"; \
#     fi && \
#     groupadd --gid $USER_GID $USERNAME && \
#     useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
#     echo "Created new user $USERNAME"

# # Add sudo support for the non-root user
# RUN apt update && apt install -y sudo \
#   && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#   && chmod 0440 /etc/sudoers.d/$USERNAME \
#   && rm -rf /var/lib/apt/lists/*

# # Switch from root to user
# USER $USERNAME
 

# # Install Git
# RUN apt update &&  apt upgrade -y \
#     apt install -y git

RUN apt-get update &&  apt-get install -y --no-install-recommends \
    git \
    curl \    
    lsb-release \
    # sudo \
    software-properties-common \
    wget \
    &&  rm -rf /var/lib/apt/lists/*

# Install python dev
RUN  apt-get install python-dev &&  apt-get install python3-dev

# Install Glib and Cmake
RUN   apt update &&   apt install \
    build-essential \
    g++ \
    libglib2.0-dev \
    cmake \
    libboost-all-dev \
    libmsgpack*

# Install lcm >=1.4.0
RUN wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip && \
    unzip v1.5.0.zip && \
    mv lcm-1.5.0 lcm && \
    cd lcm && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
      make install && \
    cd lcm-python && \
      python3 setup.py install

WORKDIR /
COPY . .

CMD ["/build.sh"]