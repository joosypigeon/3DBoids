#!/usr/bin/env bash

set -euo pipefail

echo "Installing Docker Engine with Compose and Buildx..."

# Work out the real user, even if the script is run with sudo
if [[ "${SUDO_USER:-}" != "" ]]; then
    TARGET_USER="$SUDO_USER"
else
    TARGET_USER="$USER"
fi

if [[ "$TARGET_USER" == "root" ]]; then
    echo "Warning: target user appears to be root."
    echo "Docker will be installed, but no normal user will be added to the docker group."
fi

# Require root privileges
if [[ "$EUID" -ne 0 ]]; then
    echo "Please run this script with sudo:"
    echo "  sudo ./install-docker-full.sh"
    exit 1
fi

# Check for apt-based system
if ! command -v apt >/dev/null 2>&1; then
    echo "This script is intended for Debian/Ubuntu-style apt-based distributions."
    exit 1
fi

# Load OS information
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
else
    echo "Cannot find /etc/os-release."
    exit 1
fi

echo "Detected system: ${PRETTY_NAME:-unknown}"

# Remove old/conflicting packages.
# These may or may not be installed, so failures are ignored.
echo "Removing old/conflicting Docker packages if present..."

apt remove -y \
    docker.io \
    docker-doc \
    docker-compose \
    docker-compose-v2 \
    podman-docker \
    containerd \
    runc \
    2>/dev/null || true

# Install prerequisites
echo "Installing prerequisites..."

apt update
apt install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Decide which Docker repo to use.
# Docker has separate repos for Debian and Ubuntu.
case "${ID:-}" in
    debian)
        DOCKER_DISTRO="debian"
        DOCKER_CODENAME="${VERSION_CODENAME:-}"
        ;;
    ubuntu)
        DOCKER_DISTRO="ubuntu"
        DOCKER_CODENAME="${VERSION_CODENAME:-}"
        ;;
    *)
        # For Debian-derived distributions, try to use ID_LIKE.
        # Linux Mint, Pop!_OS, etc. are usually Ubuntu-based.
        # LMDE and similar are Debian-based.
        if echo "${ID_LIKE:-}" | grep -qi "ubuntu"; then
            DOCKER_DISTRO="ubuntu"
            DOCKER_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
        elif echo "${ID_LIKE:-}" | grep -qi "debian"; then
            DOCKER_DISTRO="debian"
            DOCKER_CODENAME="${DEBIAN_CODENAME:-${VERSION_CODENAME:-}}"
        else
            echo "Unsupported or unrecognised distribution: ${ID:-unknown}"
            echo "This script supports Debian/Ubuntu and close derivatives."
            exit 1
        fi
        ;;
esac

if [[ -z "$DOCKER_CODENAME" ]]; then
    echo "Could not determine distribution codename."
    echo "For Debian 13 this should be: trixie"
    echo "For Debian 12 this should be: bookworm"
    echo "For Ubuntu 24.04 this should be: noble"
    exit 1
fi

echo "Using Docker repository:"
echo "  Distro:   $DOCKER_DISTRO"
echo "  Codename: $DOCKER_CODENAME"

# Add Docker's official GPG key
echo "Adding Docker GPG key..."

install -m 0755 -d /etc/apt/keyrings

curl -fsSL "https://download.docker.com/linux/${DOCKER_DISTRO}/gpg" \
    -o /etc/apt/keyrings/docker.asc

chmod a+r /etc/apt/keyrings/docker.asc

# Add Docker repository
echo "Adding Docker APT repository..."

ARCH="$(dpkg --print-architecture)"

cat > /etc/apt/sources.list.d/docker.list <<EOF
deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/${DOCKER_DISTRO} ${DOCKER_CODENAME} stable
EOF

# Install Docker and useful plugins
echo "Installing Docker packages..."

apt update
apt install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

# Enable and start Docker
echo "Enabling and starting Docker service..."

systemctl enable --now docker

# Add user to docker group
echo "Setting up non-root Docker access..."

groupadd -f docker

if [[ "$TARGET_USER" != "root" ]]; then
    usermod -aG docker "$TARGET_USER"
    echo "Added user '$TARGET_USER' to the docker group."
fi

# Basic checks
echo
echo "Installed versions:"
docker --version || true
docker compose version || true
docker buildx version || true

echo
echo "Testing Docker with hello-world..."
docker run --rm hello-world

echo
echo "Docker installation complete."
echo
echo "Important:"
echo "  Log out and log back in for docker group membership to take effect."
echo "  Or run this in your current terminal:"
echo
echo "      newgrp docker"
echo
echo "Then test without sudo:"
echo
echo "      docker run --rm hello-world"
echo
echo "Security note:"
echo "  Users in the docker group effectively have root-level control of this machine."
