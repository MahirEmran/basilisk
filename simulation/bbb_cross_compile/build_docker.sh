#!/bin/bash

# Docker-based build script for BeagleBone Black cross-compilation on macOS
# This script uses Docker to provide a Linux environment with ARM cross-compiler

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_DIR="${SCRIPT_DIR}/install"
DOCKER_IMAGE="bbb-cross-compile"
DOCKER_CONTAINER="bbb-build-container"

# Print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is available
check_docker() {
    print_info "Checking for Docker..."

    if ! command -v docker &> /dev/null; then
        print_error "Docker not found!"
        print_info "Please install Docker Desktop for Mac from https://www.docker.com/products/docker-desktop"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        print_error "Docker is not running!"
        print_info "Please start Docker Desktop"
        exit 1
    fi

    print_success "Docker is available and running"
}

# Build Docker image
build_docker_image() {
    print_info "Building Docker image for cross-compilation..."

    if docker images | grep -q "${DOCKER_IMAGE}"; then
        print_warning "Docker image already exists, skipping build"
        return
    fi

    docker build -t "${DOCKER_IMAGE}" "${SCRIPT_DIR}"
    print_success "Docker image built successfully"
}

# Clean build directory
clean_build() {
    print_info "Cleaning build directory..."
    rm -rf "${BUILD_DIR}"
    rm -rf "${INSTALL_DIR}"
    print_success "Build directory cleaned"
}

# Configure CMake inside Docker
configure_cmake() {
    print_info "Configuring CMake inside Docker..."

    mkdir -p "${BUILD_DIR}"
    mkdir -p "${INSTALL_DIR}"

    docker run --rm \
        -v "${SCRIPT_DIR}:/workspace" \
        -w /workspace \
        "${DOCKER_IMAGE}" \
        bash -c "cd build && \
        export PKG_CONFIG_PATH=/usr/lib/arm-linux-gnueabihf/pkgconfig:/usr/share/pkgconfig && \
        export PKG_CONFIG_LIBDIR=/usr/lib/arm-linux-gnueabihf/pkgconfig && \
        cmake -DCMAKE_TOOLCHAIN_FILE=../bbb_toolchain.cmake \
              -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
              -DCMAKE_INSTALL_PREFIX=/workspace/install \
              .."

    print_success "CMake configuration completed"
}

# Build the project inside Docker
build_project() {
    print_info "Building project inside Docker (${BUILD_TYPE})..."

    docker run --rm \
        -v "${SCRIPT_DIR}:/workspace" \
        -w /workspace/build \
        "${DOCKER_IMAGE}" \
        bash -c "cmake --build . --config ${BUILD_TYPE} -j\$(nproc)"

    print_success "Build completed successfully"
}

# Install the project inside Docker
install_project() {
    print_info "Installing project inside Docker..."

    docker run --rm \
        -v "${SCRIPT_DIR}:/workspace" \
        -w /workspace/build \
        "${DOCKER_IMAGE}" \
        bash -c "cmake --install ."

    print_success "Installation completed"
    print_info "Installed files are in: ${INSTALL_DIR}"
}

# Show build summary
show_summary() {
    print_info "Build Summary:"
    echo "  Build Type: ${BUILD_TYPE}"
    echo "  Build Directory: ${BUILD_DIR}"
    echo "  Install Directory: ${INSTALL_DIR}"
    echo ""
    print_info "Installed Files:"
    if [ -d "${INSTALL_DIR}" ]; then
        find "${INSTALL_DIR}" -type f | sort
    fi
}

# Main build process
main() {
    print_info "Starting BeagleBone Black cross-compilation build (Docker)..."
    echo ""

    # Parse command line arguments
    CLEAN_BUILD=false
    BUILD_TYPE="Release"
    REBUILD_DOCKER=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                CLEAN_BUILD=true
                shift
                ;;
            --debug)
                BUILD_TYPE="Debug"
                shift
                ;;
            --release)
                BUILD_TYPE="Release"
                shift
                ;;
            --rebuild-docker)
                REBUILD_DOCKER=true
                shift
                ;;
            --help)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --clean          Clean build directory before building"
                echo "  --debug          Build in Debug mode"
                echo "  --release        Build in Release mode (default)"
                echo "  --rebuild-docker Rebuild Docker image"
                echo "  --help           Show this help message"
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Check Docker
    check_docker

    # Rebuild Docker image if requested
    if [ "$REBUILD_DOCKER" = true ]; then
        print_info "Rebuilding Docker image..."
        docker rmi "${DOCKER_IMAGE}" 2>/dev/null || true
    fi

    # Build Docker image
    build_docker_image

    # Clean build if requested
    if [ "$CLEAN_BUILD" = true ]; then
        clean_build
    fi

    # Build process
    configure_cmake
    build_project
    install_project

    # Show summary
    echo ""
    show_summary

    print_success "Build process completed successfully!"
    echo ""
    print_info "Next steps:"
    print_info "1. Transfer files to BeagleBone Black:"
    print_info "   scp -r ${INSTALL_DIR}/* debian@beaglebone:/home/debian/active_guidance/"
    print_info "2. SSH into BeagleBone Black:"
    print_info "   ssh debian@beaglebone"
    print_info "3. Run the gateway:"
    print_info "   cd /home/debian/active_guidance && ./bin/bbb_gateway"
}

# Run main function
main "$@"