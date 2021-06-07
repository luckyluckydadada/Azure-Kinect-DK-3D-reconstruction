# > Set -DBUILD_CUDA_MODULE=ON if CUDA is available (e.g. on Nvidia Jetson)
# > Set -DBUILD_GUI=ON if OpenGL is available (e.g. on Nvidia Jetson)
# > We don't support TensorFlow and PyTorch on ARM officially
cmake \
    -DBUILD_AZURE_KINECT=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_CUDA_MODULE=ON \
    -DBUILD_GUI=ON \
    -DBUILD_TENSORFLOW_OPS=OFF \
    -DBUILD_PYTORCH_OPS=OFF \
    -DBUILD_UNIT_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_JUPYTER_EXTENSION=ON \
    -DCMAKE_INSTALL_PREFIX=~/open3d_install \
    -DPYTHON_EXECUTABLE="$(which python3)" \
    ..

# Build C++ library
make -j$(nproc)

# Run tests (optional)
make tests -j$(nproc)
./bin/tests --gtest_filter="-*Reduce*Sum*"

# Install C++ package (optional)
sudo make install
