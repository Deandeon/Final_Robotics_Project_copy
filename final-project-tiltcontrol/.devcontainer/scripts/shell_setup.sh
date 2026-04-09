#!/bin/bash


if [[ -z "${USER_HOME}" ]]; then
    readonly USER_HOME="${HOME:-/home/ubuntu}"
fi
if [[ -z "${DEV_DIR}" ]]; then
    readonly DEV_DIR="${DEV_DIR:-${USER_HOME}/development}"
fi
if [[ -z "${SHELL_TYPE}" ]]; then
    readonly SHELL_TYPE="${1:-${SHELL:-zsh}}"
fi

# Logging control - set to false to disable logging
if [[ -z "${ENABLE_LOGGING}" ]]; then
    readonly ENABLE_LOGGING="${ENABLE_LOGGING:-false}"
fi

# Available ROS2 workspaces (in order of preference)
if [[ -z "${ROS_WORKSPACES}" ]]; then
    readonly ROS_WORKSPACES=(
        "ros2_ws"
    )
fi

# Detect shell if not provided
if [[ -z "$1" && -z "$SHELL" ]]; then
    if [[ -n "$ZSH_VERSION" ]]; then
        SHELL_TYPE="zsh"
    elif [[ -n "$BASH_VERSION" ]]; then
        SHELL_TYPE="bash"
    else
        SHELL_TYPE="bash"  # fallback
    fi
fi

# ==================== UTILITY FUNCTIONS ====================
log_info() {
    [[ "$ENABLE_LOGGING" == "true" ]] && echo -e "\033[0;34m[INFO]\033[0m $1"
}

log_warn() {
    [[ "$ENABLE_LOGGING" == "true" ]] && echo -e "\033[0;33m[WARN]\033[0m $1"
}

log_error() {
    [[ "$ENABLE_LOGGING" == "true" ]] && echo -e "\033[0;31m[ERROR]\033[0m $1"
}

safe_export() {
    local var_name="$1"
    local var_value="$2"

    if [[ -n "$var_value" ]]; then
        export "$var_name"="$var_value"
    fi
}

add_to_path() {
    local new_path="$1"
    if [[ -d "$new_path" && ":$PATH:" != *":$new_path:"* ]]; then
        export PATH="$new_path:$PATH"
    fi
}

# =================== ROS WORKSPACE SELECTION ===================
select_ros_workspace() {
    local provided_ws="$1"

    # If a workspace is explicitly provided and exists, use it
    if [[ -n "$provided_ws" ]]; then
        local ws_path="$DEV_DIR/ros/$provided_ws"
        if [[ -d "$ws_path" ]]; then
            echo "$provided_ws"
            return 0
        else
            log_warn "Specified workspace '$provided_ws' not found at: $ws_path" >&2
        fi
    fi

    # Try workspaces in order of preference
    for ws in "${ROS_WORKSPACES[@]}"; do
        local ws_path="$DEV_DIR/ros/$ws"
        if [[ -d "$ws_path" ]]; then
            log_info "Auto-selected workspace: $ws" >&2
            echo "$ws"
            return 0
        fi
    done

    # Fallback to first workspace in list (even if it doesn't exist)
    local fallback_ws="${ROS_WORKSPACES[0]}"
    log_warn "No existing workspace found, using fallback: $fallback_ws" >&2
    echo "$fallback_ws"
}

# ==================== CORE ENVIRONMENT SETUP ====================
setup_core_environment() {
    log_info "Setting up core environment..."

    # Basic PATH setup
    add_to_path "${USER_HOME}/.local/bin"
    add_to_path "/usr/local/bin"

}


# ==================== ROS2 SETUP ====================
setup_ros() {
    log_info "Setting up ROS2 environment..."

    # ROS2 Configuration
    local ros2_distro="${ROS2_DISTRO:-jazzy}"
    local ros_domain_id="${ROS_DOMAIN_ID:-5}"
    local gz_version="${GZ_VERSION:-harmonic}"
    local rmw_implementation="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

    # Smart workspace selection
    local default_ws
    default_ws="$(select_ros_workspace "$WS")"

    # Export ROS2 environment variables
    safe_export "ROS2_DISTRO" "$ros2_distro"
    safe_export "ROS_DISTRO" "$ros2_distro"
    safe_export "ROS_DOMAIN_ID" "$ros_domain_id"
    safe_export "GZ_VERSION" "$gz_version"
    safe_export "RMW_IMPLEMENTATION" "$rmw_implementation"
    safe_export "WS" "$default_ws"

    # Check ROS2 installation
    local ros_setup_file="/opt/ros/$ros2_distro/setup.$SHELL_TYPE"
    if [[ ! -f "$ros_setup_file" ]]; then
        log_error "ROS2 $ros2_distro not found. Please install ROS2 $ros2_distro"
        return 1
    fi

    # Source ROS2 setup files
    source "$ros_setup_file"

    # Colcon setup
    local colcon_cd_func="/usr/share/colcon_cd/function/colcon_cd.sh"
    local colcon_argcomplete="/usr/share/colcon_argcomplete/hook/colcon-argcomplete.$SHELL_TYPE"
    local colcon_cd_argcomplete="/usr/share/colcon_cd/function/colcon_cd-argcomplete.$SHELL_TYPE"

    [[ -f "$colcon_cd_func" ]] && source "$colcon_cd_func"
    [[ -f "$colcon_argcomplete" ]] && source "$colcon_argcomplete"
    [[ -f "$colcon_cd_argcomplete" ]] && source "$colcon_cd_argcomplete"

    safe_export "_colcon_cd_root" "/opt/ros/$ros2_distro/"

    # Source workspace setup if it exists
    local ws_setup_file="$DEV_DIR/ros/$default_ws/install/setup.$SHELL_TYPE"
    if [[ -f "$ws_setup_file" ]]; then
        source "$ws_setup_file"
        log_info "Workspace '$default_ws' sourced successfully"
    else
        log_warn "Workspace setup file not found: $ws_setup_file"
    fi

    # Setup argcomplete for ros2 and colcon
    if command -v register-python-argcomplete >/dev/null 2>&1; then
        eval "$(register-python-argcomplete ros2)"
        eval "$(register-python-argcomplete colcon)"
    fi

    # ROS2-specific aliases
    alias rd="cd \$DEV_DIR/ros/\$WS"
    alias rds="cd \$DEV_DIR/ros/\$WS/src"
    alias drd='rd && sudo rm -rf build log install'

    # ROS2 build aliases
    alias cbs='rd && colcon build --symlink-install && sr'
    alias cb='rd && colcon build && sr'
    alias cbp='rd && colcon build --packages-select'
    alias cbsp='rd && colcon build --symlink-install --packages-select'

    # ROS2 dependency and utility aliases
    alias ri='rd && rosdep install --from-paths src --ignore-src -y -r'
    alias rr='ros2 run'
    alias rl='ros2 launch'
    alias rtl='ros2 topic list'
    alias rnl='ros2 node list'
    alias rte='ros2 topic echo'
    alias rbr='ros2 bag record'
    alias rdr='ros2 doctor --report'
    alias ct='rd && colcon test --packages-select'

    alias rpp="rds && ros2 pkg create --dependencies rclpy --build-type ament_python"
    alias rpc="rds && ros2 pkg create --dependencies rclcpp --build-type ament_cmake"
    alias rpph="ros2 pkg create --dependencies rclpy --build-type ament_python"
    alias rpch="ros2 pkg create --dependencies rclcpp --build-type ament_cmake"

    log_info "ROS2 environment configured for workspace: $default_ws"
}

# ==================== CORE ALIASES SETUP ====================
setup_core_aliases() {
    log_info "Setting up core aliases..."

    # Shell reloading aliases
    case "$SHELL_TYPE" in
        bash)
            alias sr='source ~/.bashrc'
            ;;
        zsh)
            alias sr='source ~/.zshrc'
            autoload -U +X bashcompinit && bashcompinit
            ;;
    esac

    # System configuration aliases
    alias snb='sudo nano ~/.bashrc'
    alias snz='sudo nano ~/.zshrc'
    alias snm="nano \$DEV_DIR/scripts/my_setup.sh"
    alias dd='cd $DEV_DIR'

    # Utility script aliases
    alias krn="source \$DEV_DIR/scripts/kill_ros.sh"
    alias dl="source \$DEV_DIR/scripts/delock.sh"

    # Git aliases
    alias gsu='git submodule sync --recursive && git submodule update --init --recursive'
    alias gc='git clone'
    alias gpb='git remote prune origin && git branch -d'

    # System package management aliases
    alias ud='sudo apt-get update -y'
    alias ug='sudo apt-get upgrade -y'
    alias udg='ud && ug'
    alias si='sudo apt-get install -y'
    alias sdi='sudo dpkg -i'

}

# ==================== PYTHON TOOLS SETUP ====================
setup_python_tools() {
    log_info "Setting up Python tools..."

    # UV configuration
    if command -v uv >/dev/null 2>&1; then
        eval "$(uv generate-shell-completion $SHELL_TYPE)"
        log_info "UV shell completion configured"

        # UV-specific aliases
        alias uva='uv add'
        alias uvv='uv venv'
        alias uvi='uv init'
        alias uvpi='uv pip install'
        alias uvr='uv remove'
    fi

    # Python aliases
    alias python='python3'
    alias py='python3'

    # Python package management aliases
    alias pi='pip install --break-system-packages'
    alias pi3='pip3 install --break-system-packages'
}

setup_optional_components() {
    # Python Tools Setup
    if [[ "${ENABLE_PYTHON_TOOLS:-true}" == "true" ]]; then
        setup_python_tools
    else
        log_info "Python tools setup disabled (ENABLE_PYTHON_TOOLS=false)"
    fi

    # ROS2 Setup
    if [[ "${ENABLE_ROS:-true}" == "true" ]]; then
        setup_ros
    else
        log_info "ROS2 setup disabled (ENABLE_ROS=false)"
    fi
}


# ==================== INTERACTIVE FUNCTIONS ====================
# List available ROS workspaces
lws() {
    log_info "Available ROS2 workspaces:"
    local current_ws="${WS:-none}"

    for ws in "${ROS_WORKSPACES[@]}"; do
        local ws_path="$DEV_DIR/ros/$ws"
        local ws_status="✗"
        local marker=""

        if [[ -d "$ws_path" ]]; then
            ws_status="✓"
        fi

        if [[ "$ws" == "$current_ws" ]]; then
            marker=" (current)"
        fi

        echo "  $ws_status $ws$marker"
    done

    echo ""
    echo "Usage: crws <workspace_name> - switch workspace"
    echo "       lws                   - list workspaces"
}

# Change ROS workspace interactively
crws() {
    local new_ws="$1"

    # If no argument provided, show available workspaces
    if [[ -z "$new_ws" ]]; then
        log_info "Available workspaces:"
        lws
        echo ""
        log_error "Usage: crws <workspace_name>"
        return 1
    fi

    # Check if it's a valid workspace name
    local valid_workspace=false
    for ws in "${ROS_WORKSPACES[@]}"; do
        if [[ "$ws" == "$new_ws" ]]; then
            valid_workspace=true
            break
        fi
    done

    if [[ "$valid_workspace" == "false" ]]; then
        log_error "Invalid workspace name: $new_ws"
        log_info "Valid workspaces are: ${ROS_WORKSPACES[*]}"
        return 1
    fi

    local ws_path="$DEV_DIR/ros/$new_ws"
    if check_directory "$ws_path" "Workspace '$new_ws'"; then
        export WS="$new_ws"
        log_info "Switched to workspace: $new_ws"
        sr  # Reload shell configuration
    else
        log_error "Workspace '$new_ws' not found at: $ws_path"
        log_info "You can create it with: mkdir -p $ws_path/src"
        return 1
    fi
}

# Set ROS to localhost only mode
rlh() {
    export ROS_DOMAIN_ID=5
    export ROS_LOCALHOST_ONLY=1
    log_info "ROS configured for localhost only mode"
    printenv | grep ROS_
}

# ==================== CONVENIENCE FUNCTIONS ====================
# Function to enable quiet mode (disable logging)
quiet_mode() {
    export ENABLE_LOGGING=false
    echo "Quiet mode enabled - logging disabled"
}

# Function to enable verbose mode (enable logging)
verbose_mode() {
    export ENABLE_LOGGING=true
    log_info "Verbose mode enabled - logging enabled"
}

# ==================== MAIN SETUP EXECUTION ====================
main() {
    log_info "Starting development environment setup for shell: $SHELL_TYPE"

    # Core setup (always enabled)
    setup_core_environment
    setup_core_aliases

    # Optional components (can be disabled via environment variables)
    setup_optional_components

    log_info "Development environment setup completed successfully!"
    if [[ "${ENABLE_ROS:-true}" == "true" ]]; then
        # Always show current workspace regardless of logging mode
        echo -e "\033[0;34m[INFO]\033[0m Current ROS2 workspace: ${WS:-not set}"
    fi
}

# ==================== COMPONENT ENABLEMENT FLAGS ====================
# Set these environment variables to control which components are loaded
export ENABLE_ROS=true           # Enable ROS2 setup
export ENABLE_PYTHON_TOOLS=true  # Enable Python tools setup


main