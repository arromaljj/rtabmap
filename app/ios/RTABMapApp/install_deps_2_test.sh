#!/bin/bash

set -euxo pipefail

# Tested on Apple Silicon Mac, with cmake 3.19.2.

mkdir -p Libraries && cd "$_"

# Set common variables
prefix=$(pwd)
sysroot=iphoneos  # Use iphoneos, change to iphonesimulator if needed

# Function to build a library
build_library() {
  local library_name="$1"
  local source_dir="$2"
  local include_dir="$3"
  local dependencies=()
  local cmake_options=()

  # Check if library is already installed
  if [ ! -e "$prefix/include/$include_dir" ]; then
    # Download and extract if necessary
    if [ ! -e "$source_dir" ]; then
      echo "Downloading $library_name..."
      curl -L "${library_name}_download_url" -o "${library_name}.tar.gz"
      tar -xzf "${library_name}.tar.gz"
    fi

    cd "$source_dir"

    # Apply patches if needed
    if [ ! -e "${library_name}_ios.patch" ]; then
      curl -L "${library_name}_patch_url" -o "${library_name}_ios.patch"
      git apply "${library_name}_ios.patch"
    fi

    mkdir -p build && cd "$_"

    # Configure CMake
    cmake -G Xcode \
          -DCMAKE_SYSTEM_NAME=iOS \
          -DCMAKE_OSX_ARCHITECTURES=arm64 \
          -DCMAKE_OSX_SYSROOT="$sysroot" \
          -DBUILD_SHARED_LIBS=OFF \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
          -DCMAKE_INSTALL_PREFIX="$prefix" \
          "${dependencies[@]}" \
          "${cmake_options[@]}" \
          ".."

    cmake --build . --config Release
    cmake --build . --config Release --target install

    cd "$pwd"
    # Cleanup (optional)
    # rm -rf "${library_name}.tar.gz" "$source_dir"
  fi
}

# Define library configurations
declare -A libraries=(
  ["boost"]=(
    "download_url"="https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz"
    "source_dir"="boost_1_59_0"
    "include_dir"="boost"
    "cmake_options"=(
      "-DBUILD_SHARED_LIBS=OFF"
    )
  )
  ["eigen"]=(
    "download_url"="https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz"
    "source_dir"="eigen-3.3.9"
    "include_dir"="eigen3"
  )
  ["flann"]=(
    "download_url"=""  # Use git clone
    "source_dir"="flann"
    "include_dir"="flann"
    "cmake_options"=(
      "-DBUILD_PYTHON_BINDINGS=OFF"
      "-DBUILD_MATLAB_BINDINGS=OFF"
      "-DBUILD_C_BINDINGS=OFF"
    )
  )
  ["gtsam"]=(
    "download_url"=""  # Use git clone
    "source_dir"="gtsam"
    "include_dir"="gtsam"
    "git_branch"="fbb9d3bdda8b88df51896bc401bfd170573e66f5"
    "patch_url"="https://gist.githubusercontent.com/matlabbe/76d658dddb841b3355ae3a6e32850cd8/raw/7033cba1c89097b0c830651d7277c04dc92cbdd9/gtsam_GKlib_ios_fix.patch"
    "cmake_options"=(
      "-DMETIS_SHARED=OFF"
      "-DGTSAM_BUILD_STATIC_LIBRARY=ON"
      "-DGTSAM_BUILD_TESTS=OFF"
      "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
      "-DGTSAM_USE_SYSTEM_EIGEN=ON"
      "-DGTSAM_WRAP_SERIALIZATION=OFF"
      "-DGTSAM_BUILD_WRAP=OFF"
      "-DGTSAM_INSTALL_CPPUNITLITE=OFF"
    )
  )
  # ... (other libraries)
)

# Build each library
for library_name in "${!libraries[@]}"; do
  local library_config="${libraries[$library_name]}"
  build_library "$library_name" "${library_config[source_dir]}" "${library_config[include_dir]}" "${library_config[dependencies[@]]}" "${library_config[cmake_options[@]]}"
done

# Build Rtabmap
mkdir -p rtabmap && cd "$_"
cmake -DANDROID_PREBUILD=ON ../../../../..
cmake --build . --config Release
mkdir -p ios && cd "$_"
cmake -G Xcode \
      -DCMAKE_SYSTEM_NAME=iOS \
      -DCMAKE_OSX_ARCHITECTURES=arm64 \
      -DCMAKE_OSX_SYSROOT="$sysroot" \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
      -DCMAKE_INSTALL_PREFIX="$prefix" \
      -DCMAKE_FIND_ROOT_PATH="$prefix" \
      -DWITH_QT=OFF \
      -DBUILD_APP=OFF \
      -DBUILD_TOOLS=OFF \
      -DWITH_TORO=OFF \
      -DWITH_VERTIGO=OFF \
      -DWITH_MADGWICK=OFF \
      -DWITH_ORB_OCTREE=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DWITH_LIBLAS=ON \
      ../../../../../..
cmake --build . --config Release
cmake --build . --config Release --target install