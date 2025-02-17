FROM nvidia/cudagl:11.3.1-base-ubuntu20.04

ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV USERNAME=cudagl

# Set timezone and noninteractive mode
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Add user and set working directory
RUN useradd -m ${USERNAME} && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    usermod --shell /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME}
WORKDIR /home/${USERNAME}

# Optimize the apt mirror for faster package downloads
RUN sed -i.bak 's|http://archive.ubuntu.com/ubuntu/|http://mirror.math.princeton.edu/pub/ubuntu/|' /etc/apt/sources.list

# Set up timezone configuration to avoid interactive prompts
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y --no-install-recommends \
    tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install basic tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo less tmux bash-completion curl wget coreutils git \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install OpenGL-related libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dev freeglut3-dev mesa-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install archive tools (xz-utils)
RUN apt-get update && apt-get install -y --no-install-recommends \
    xz-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install build tools and dependencies for Python build
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential zlib1g-dev libffi-dev libssl-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up temporary XDG_RUNTIME_DIR
RUN mkdir -p /tmp/xdg_runtime_dir && chmod 777 /tmp/xdg_runtime_dir
RUN echo "export XDG_RUNTIME_DIR=/tmp/xdg_runtime_dir" >> ~/.bashrc


#####################################################
# 		   	     Python 3.10.12
#####################################################
# Install dependencies for Tkinter
RUN apt-get update && apt-get install -y --no-install-recommends \
    libx11-dev libxext-dev libxft-dev libxss-dev \
    libxrandr-dev tk-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install SQLite development libraries (required for _sqlite3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libsqlite3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python 3.10.12 with Tkinter support
RUN curl -O https://www.python.org/ftp/python/3.10.12/Python-3.10.12.tar.xz \
    && tar -xf Python-3.10.12.tar.xz \
    && cd Python-3.10.12 \
    && ./configure --enable-optimizations --with-tcltk \
    && make -j8 \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.10.12 Python-3.10.12.tar.xz

# Set up symbolic links for python and pip
RUN ln -sf /usr/local/bin/python3.10 /usr/local/bin/python
RUN ln -sf /usr/local/bin/pip3.10 /usr/local/bin/pip

# Upgrade pip
RUN pip install --upgrade pip

# Install Tkinter for Python GUI applications
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC apt-get install -y --no-install-recommends \
    python3-tk \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

#####################################################
# 		   		   Mujoco 3.2.3
#####################################################
RUN apt-get update \
    && apt-get install -y manpages-dev psmisc tree\
    # Install gl/x package for visualization inside docker
    && apt-get install -y libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 libzmq3-dev
RUN pip install mujoco==3.2.3


# Switch to non-root user
USER $USERNAME
CMD ["/bin/bash"]
