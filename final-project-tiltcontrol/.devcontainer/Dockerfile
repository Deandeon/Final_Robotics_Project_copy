FROM osrf/ros:jazzy-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEV_FOLDER=/home/${USERNAME}/development
ENV ROS2_WS=${DEV_FOLDER}/ros/ros2_ws
ENV SCRIPTS_FOLDER=${DEV_FOLDER}/scripts

ENV TIMEZONE=Africa/Accra

RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Create necessary folders
RUN mkdir -p ${ROS2_WS}/src ${SCRIPTS_FOLDER}  && \
    sudo chown -R ${USERNAME}:${USERNAME} ${DEV_FOLDER}

    # Create XDG runtime directory
RUN sudo mkdir -p /tmp/runtime-${USERNAME} && \
    sudo chown ${USERNAME}:${USERNAME} /tmp/runtime-${USERNAME} && \
    sudo chmod 700 /tmp/runtime-${USERNAME}

ENV XDG_RUNTIME_DIR=/tmp/runtime-${USERNAME}

# Copy scripts folder contents
COPY --chown=${USERNAME}:${USERNAME} scripts/ ${SCRIPTS_FOLDER}/


# Install dependencies
RUN sudo apt-get update -q -y --fix-missing && \
    sudo apt-get upgrade -q -y && \
    sudo apt-get install -y software-properties-common locales lsb-release \
    ros-dev-tools python3 python3-dev python3-pip python3-colcon-common-extensions python3-rosdep \
    bash-completion xauth tzdata git wget curl nano zsh fonts-powerline powerline

RUN sudo pip install --break-system-packages -U ruff

# Set Locale
RUN sudo locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

################################ Not Compulsory: Terminal setup
# Install Oh My Zsh (non-interactive)
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Install Spaceship Prompt as an Oh My Zsh custom theme
RUN git clone https://github.com/spaceship-prompt/spaceship-prompt.git \
    ~/.oh-my-zsh/custom/themes/spaceship-prompt --depth=1 && \
    ln -s ~/.oh-my-zsh/custom/themes/spaceship-prompt/spaceship.zsh-theme \
    ~/.oh-my-zsh/custom/themes/spaceship.zsh-theme

WORKDIR /home/${USERNAME}/.oh-my-zsh/custom/plugins
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting && \
    git clone https://github.com/zsh-users/zsh-autosuggestions

WORKDIR /home/${USERNAME}

# Change default shell to zsh for the user
RUN sudo chsh -s $(which zsh) $USERNAME


# Configure Zsh theme and plugins
RUN sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="spaceship"/g' ~/.zshrc
RUN sed -i.bak 's/^plugins=(git.*/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' ~/.zshrc

# Set Timezone
RUN sudo ln -sf /usr/share/zoneinfo/${TIMEZONE} /etc/localtime && \
sudo dpkg-reconfigure -f noninteractive tzdata

################################# END Terminal setup

# Source setup script in bashrc
RUN echo "source ${SCRIPTS_FOLDER}/shell_setup.sh zsh" >> ~/.zshrc
RUN echo "source ${SCRIPTS_FOLDER}/shell_setup.sh bash" >> ~/.bashrc
