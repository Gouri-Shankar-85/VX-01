#!/bin/bash
set -e

[ -f ".env" ] && source .env

REGISTRY="${DOCKER_REGISTRY:-yourdockerhub}"
TAG="humble"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
info() { echo -e "${BLUE}[i]${NC} $1"; }
ok()   { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
die()  { echo -e "${RED}[✗]${NC} $1"; exit 1; }

usage() {
cat << 'EOF'
Usage: ./build.sh COMMAND

  setup-x11        Allow Docker to open GUI windows (run once per login session)
  udev-install     Install device rules on this host (requires sudo, run once ever)
  build-base       Build hardware image for AMD64 (laptop testing)
  build-sim        Build simulation image for AMD64 (laptop only)
  build-dashboard  Build dashboard image (AMD64 + push to Docker Hub)
  build-arm        Cross-compile hardware image for ARM64 via QEMU (may segfault)
  build-arm-ci     Trigger ARM64 build via GitHub Actions (recommended, no QEMU)
  push             Push all AMD64 images to Docker Hub
  full             Do everything: udev + x11 + all builds + push

EOF
}

setup_x11() {
    info "Enabling X11 for Docker GUI..."
    xhost +local:docker
    ok "Done. Add 'xhost +local:docker' to ~/.bashrc to make it permanent."
}

udev_install() {
    info "Installing udev rules on this host..."
    sudo cp docker/udev/99-vx01-devices.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sudo usermod -aG dialout,plugdev,video,i2c "${USER}" 2>/dev/null || true
    ok "udev rules installed. Log out and back in for group changes to take effect."
}

build_base() {
    info "Building base image (AMD64)..."
    docker build \
        --platform linux/amd64 \
        -f docker/Dockerfile.base \
        -t vx01-base:${TAG} \
        -t ${REGISTRY}/vx01-base:${TAG}-amd64 \
        .
    ok "vx01-base:${TAG} ready"
}

build_sim() {
    docker image inspect vx01-base:${TAG} &>/dev/null || build_base
    info "Building simulation image (AMD64)..."
    docker build \
        --platform linux/amd64 \
        -f docker/Dockerfile.sim \
        -t vx01-sim:${TAG} \
        -t ${REGISTRY}/vx01-sim:${TAG} \
        .
    ok "vx01-sim:${TAG} ready"
}

build_dashboard() {
    [ "${REGISTRY}" = "yourdockerhub" ] && \
        die "Set DOCKER_REGISTRY in .env to your Docker Hub username first"
    info "Building dashboard image..."
    docker build \
        --platform linux/amd64 \
        -f docker/Dockerfile.dashboard \
        -t vx01-dashboard:latest \
        -t ${REGISTRY}/vx01-dashboard:latest \
        .
    info "Pushing dashboard image..."
    docker push ${REGISTRY}/vx01-dashboard:latest
    ok "Dashboard image pushed: ${REGISTRY}/vx01-dashboard:latest"
}

build_arm() {
    [ "${REGISTRY}" = "yourdockerhub" ] && \
        die "Set DOCKER_REGISTRY in .env to your Docker Hub username first"

    info "Cross-compiling for ARM64 via QEMU..."
    warn "If this segfaults, run: ./build.sh build-arm-ci  (uses GitHub Actions)"

    docker buildx use vx01-builder 2>/dev/null || \
        docker buildx create --name vx01-builder \
            --driver-opt env.BUILDKIT_STEP_LOG_MAX_SIZE=50000000 \
            --use
    docker buildx inspect --bootstrap

    MAX_RETRIES=3
    attempt=1
    while [ $attempt -le $MAX_RETRIES ]; do
        info "Build attempt ${attempt}/${MAX_RETRIES}..."
        EXTRA_FLAGS=""
        [ $attempt -gt 1 ] && EXTRA_FLAGS="--no-cache" && warn "Retrying with --no-cache"

        if docker buildx build \
            --platform linux/arm64 \
            -f docker/Dockerfile.base \
            -t ${REGISTRY}/vx01-base:${TAG}-arm64 \
            --shm-size=512m \
            --push \
            ${EXTRA_FLAGS} \
            .; then
            ok "ARM64 image pushed: ${REGISTRY}/vx01-base:${TAG}-arm64"
            return 0
        fi

        warn "Attempt ${attempt} failed."
        attempt=$((attempt + 1))
        [ $attempt -le $MAX_RETRIES ] && info "Waiting 10s before retry..." && sleep 10
    done

    die "All ${MAX_RETRIES} attempts failed. Run './build.sh build-arm-ci' to use GitHub Actions instead."
}

build_arm_ci() {
    [ "${REGISTRY}" = "yourdockerhub" ] && \
        die "Set DOCKER_REGISTRY in .env to your Docker Hub username first"

    if ! command -v gh &>/dev/null; then
        die "GitHub CLI (gh) not installed. Install: https://cli.github.com\n   Then run: gh auth login"
    fi

    info "Triggering ARM64 build on GitHub Actions..."
    gh workflow run build-arm.yml

    ok "Build triggered. Monitor at: https://github.com/Gouri-Shankar-85/VX-01/actions"
    info "Once complete, pull the image on your Pi 5:"
    info "  docker pull ${REGISTRY}/vx01-base:${TAG}-arm64"
}

push_images() {
    [ "${REGISTRY}" = "yourdockerhub" ] && \
        die "Set DOCKER_REGISTRY in .env to your Docker Hub username first"
    docker login
    docker push ${REGISTRY}/vx01-base:${TAG}-amd64
    docker push ${REGISTRY}/vx01-sim:${TAG}
    docker push ${REGISTRY}/vx01-dashboard:latest
    ok "All images pushed"
}

case "${1}" in
    setup-x11)        setup_x11 ;;
    udev-install)     udev_install ;;
    build-base)       build_base ;;
    build-sim)        build_sim ;;
    build-dashboard)  build_dashboard ;;
    build-arm)        build_arm ;;
    build-arm-ci)     build_arm_ci ;;
    push)             push_images ;;
    full)
        udev_install
        setup_x11
        build_base
        build_sim
        build_dashboard
        build_arm
        push_images
        ok "All done."
        ;;
    *) usage ;;
esac