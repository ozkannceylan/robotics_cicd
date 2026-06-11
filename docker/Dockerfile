# syntax=docker/dockerfile:1
#
# Multi-stage build (ADR-5). Builder compiles the workspace against the MuJoCo
# C SDK; the slim runner ships only the install tree + MuJoCo runtime libs.
# MUJOCO_VERSION is the single source of truth for the MuJoCo version (Hard Rule #6).
ARG ROS_DISTRO=humble
ARG MUJOCO_VERSION=3.3.7

# ----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO
ARG MUJOCO_VERSION
ENV MUJOCO_DIR=/opt/mujoco-${MUJOCO_VERSION}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake wget ca-certificates \
      python3-colcon-common-extensions python3-rosdep \
      libgl1 libglew-dev \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo C SDK: download, unpack, expose an unversioned .so, register with ldconfig.
RUN wget -qO /tmp/mujoco.tgz \
      "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
    && tar -xzf /tmp/mujoco.tgz -C /opt \
    && ln -sf "${MUJOCO_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}" "${MUJOCO_DIR}/lib/libmujoco.so" \
    && echo "${MUJOCO_DIR}/lib" > /etc/ld.so.conf.d/mujoco.conf && ldconfig \
    && rm /tmp/mujoco.tgz

WORKDIR /ws
COPY otonav_description   src/otonav_description
COPY otonav_control       src/otonav_control
COPY otonav_nav           src/otonav_nav
COPY otonav_mujoco_bridge src/otonav_mujoco_bridge
COPY otonav_bringup       src/otonav_bringup

RUN apt-get update \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Unit tests (kinematics). SIL + full lint gate are wired into CI in Phase 2.
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && source install/setup.bash \
    && colcon test --merge-install --packages-select otonav_control \
    && colcon test-result --verbose

# ----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-core AS runner
ARG ROS_DISTRO
ARG MUJOCO_VERSION
ENV MUJOCO_DIR=/opt/mujoco-${MUJOCO_VERSION}

RUN apt-get update && apt-get install -y --no-install-recommends libgl1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder ${MUJOCO_DIR}/lib ${MUJOCO_DIR}/lib
RUN echo "${MUJOCO_DIR}/lib" > /etc/ld.so.conf.d/mujoco.conf && ldconfig

COPY --from=builder /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "otonav_bringup", "sim.launch.py", "gui:=false"]
