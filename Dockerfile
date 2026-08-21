FROM px4io/px4-dev-simulation-jammy:latest

# --- Locale (requis par ROS2) ---
RUN apt-get update && apt-get install -y --no-install-recommends locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# --- Dépôt ROS2 Humble ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# --- ROS2 Humble desktop (inclut RViz2) + outils de build ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init && rosdep update

# --- Données géodésiques MAVROS (obligatoire, sinon MAVROS plante au lancement) ---
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# --- Gazebo Harmonic (dépôt OSRF officiel, requis par PX4 pour les cibles gz_*) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    libunwind-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Pont ROS2 <-> Gazebo Harmonic ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# --- Perception : OpenCV (ArUco) + dépendances Python (CPU-only, plus léger/rapide à builder) ---
RUN pip3 install --no-cache-dir --ignore-installed sympy \
    #torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    opencv-contrib-python \
    ultralytics \
    numpy

# Créer le .bashrc avec le sourcing automatique
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /root/.bashrc


WORKDIR /workspace
#COPY entrypoint.sh /entrypoint.sh
#RUN chmod +x /entrypoint.sh
#ENTRYPOINT ["/entrypoint.sh"]