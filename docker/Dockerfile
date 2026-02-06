FROM prance:ROS2-Humble

# ============================================
# SFC Planner - Complete Dependencies
# ============================================

# Install base build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    pkg-config \
    libcdd-dev \
    libeigen3-dev \
    libgmp-dev \
    libmpfr-dev \
    && rm -rf /var/lib/apt/lists/*

# Create cdd symlink (IRIS expects /usr/include/cdd/)
RUN ln -sf /usr/include/cddlib /usr/include/cdd

# Install CasADi via pip
RUN pip3 install casadi

# Create CasADi symlinks for C++ linking
RUN ln -sf /usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so.3.7 /usr/local/lib/libcasadi.so && \
    ln -sf /usr/local/lib/python3.10/dist-packages/casadi/libcasadi.so.3.7 /usr/local/lib/libcasadi.so.3.7 && \
    ldconfig

# ============================================
# Install Mosek 10.1
# ============================================
WORKDIR /opt
RUN wget https://download.mosek.com/stable/10.1.28/mosektoolslinuxaarch64.tar.bz2 && \
    tar -xjf mosektoolslinuxaarch64.tar.bz2 && \
    rm mosektoolslinuxaarch64.tar.bz2

# Copy Mosek license (you'll need to provide this file)
COPY mosek.lic /opt/mosek/mosek.lic

# Set Mosek environment variables
ENV MOSEK_DIR=/opt/mosek/10.1/tools/platform/linuxaarch64
ENV PATH=$PATH:$MOSEK_DIR/bin
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MOSEK_DIR/bin
ENV MOSEKLM_LICENSE_FILE=/opt/mosek/mosek.lic

# ============================================
# Build IRIS from Source (with ARM64 fixes)
# ============================================
WORKDIR /opt
RUN git clone https://github.com/rdeits/iris-distro.git

# Fix 1: Modify main CMakeLists.txt
WORKDIR /opt/iris-distro/src
RUN sed -i '15i include(GNUInstallDirs)' CMakeLists.txt && \
    sed -i '46i set(CDD_INCLUDE_DIR "/usr/include/cdd")' CMakeLists.txt && \
    sed -i 's/find_package(Mosek CONFIG REQUIRED)/# find_package(Mosek CONFIG REQUIRED)/' CMakeLists.txt

# Add Mosek paths manually
RUN sed -i '18i set(MOSEK_DIR "/opt/mosek/10.1/tools/platform/linuxaarch64")\nset(MOSEK_INCLUDE_DIR "${MOSEK_DIR}/h")\nset(MOSEK_LIBRARY "${MOSEK_DIR}/bin/libmosek64.so")\ninclude_directories(${MOSEK_INCLUDE_DIR})\nlink_directories("${MOSEK_DIR}/bin")\nset(mosek_LIBRARIES "${MOSEK_LIBRARY}")' CMakeLists.txt

# Fix 2: Modify cxx/CMakeLists.txt
RUN sed -i '1i include(GNUInstallDirs)' cxx/CMakeLists.txt && \
    sed -i '2i set(MOSEK_DIR "/opt/mosek/10.1/tools/platform/linuxaarch64")\nset(MOSEK_LIBRARY "${MOSEK_DIR}/bin/libmosek64.so")' cxx/CMakeLists.txt && \
    sed -i '4i set(IRIS_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})\nset(IRIS_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})' cxx/CMakeLists.txt && \
    sed -i 's/target_link_libraries(iris_mosek iris_geometry mosek)/target_link_libraries(iris_mosek iris_geometry ${MOSEK_LIBRARY})/' cxx/CMakeLists.txt

# Fix 3: Fix deprecated Mosek API in iris_mosek.cpp
RUN sed -i 's/MSKCONST/const/g' cxx/iris_mosek.cpp && \
    sed -i '/MSK_SOL_STA_NEAR_OPTIMAL/d' cxx/iris_mosek.cpp && \
    sed -i '/MSK_SOL_STA_NEAR_DUAL_INFEAS_CER/d' cxx/iris_mosek.cpp && \
    sed -i '/MSK_SOL_STA_NEAR_PRIM_INFEAS_CER/d' cxx/iris_mosek.cpp

# Build IRIS
WORKDIR /opt/iris-distro
RUN mkdir build && cd build && \
    cmake .. \
      -DIRIS_WITH_EIGEN=OFF \
      -DIRIS_WITH_CDD=OFF \
      -DIRIS_WITH_MOSEK=OFF \
      -DIRIS_WITH_PYBIND11=OFF \
      -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 && \
    make -j4

# Install IRIS to system
RUN cp /opt/iris-distro/build/install/lib/libiris* /usr/local/lib/ && \
    cp -r /opt/iris-distro/build/install/include/iris /usr/local/include/ && \
    ldconfig

# ============================================
# Set up complete environment
# ============================================
# Add all library paths
ENV LD_LIBRARY_PATH=/opt/mosek/10.1/tools/platform/linuxaarch64/bin:/usr/local/lib:/usr/local/lib/python3.10/dist-packages/casadi:$LD_LIBRARY_PATH

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export MOSEKLM_LICENSE_FILE=/opt/mosek/mosek.lic" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/mosek/10.1/tools/platform/linuxaarch64/bin:/usr/local/lib:/usr/local/lib/python3.10/dist-packages/casadi:\$LD_LIBRARY_PATH" >> ~/.bashrc

# Set working directory
WORKDIR /home/prance/sfc_planner

# ============================================
# Done! Image ready for SFC development
# ============================================