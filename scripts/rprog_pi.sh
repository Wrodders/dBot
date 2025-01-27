# SSH into the Pi and run the build command
ssh "$PI_USER@$PI_IP" <<EOF
  cd "$TARGET_DIR"
  mkdir -p "$BUILD_DIR"  # Create build directory if it doesn't exist
  cd "$BUILD_DIR"
  cmake ..
  make
EOF