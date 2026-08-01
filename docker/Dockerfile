ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f
FROM ${ROS_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG ACTOR_PLUGIN_REV=e170a60b4732b0667b90b171dacb19e73208ccfd

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    python3-colcon-common-extensions \
    python3-jsonschema \
    ros-jazzy-nav2-bringup \
    ros-jazzy-navigation2 \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros-gz \
    ros-jazzy-slam-toolbox \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

ADD --checksum=sha256:c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063 \
    https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx \
    /opt/omokai_models/object_detection_yolox_2022nov.onnx
RUN chmod 0444 /opt/omokai_models/object_detection_yolox_2022nov.onnx

RUN if getent group "${HOST_GID}" >/dev/null; then \
      existing_group="$(getent group "${HOST_GID}" | cut -d: -f1)"; \
      groupmod --new-name omokai "${existing_group}"; \
    else \
      groupadd --gid "${HOST_GID}" omokai; \
    fi \
    && if getent passwd "${HOST_UID}" >/dev/null; then \
      existing_user="$(getent passwd "${HOST_UID}" | cut -d: -f1)"; \
      usermod --login omokai --home /home/omokai --move-home --gid "${HOST_GID}" --shell /bin/bash "${existing_user}"; \
    else \
      useradd --uid "${HOST_UID}" --gid "${HOST_GID}" --create-home --shell /bin/bash omokai; \
    fi

ENV ROS_DISTRO=jazzy \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    TURTLEBOT3_MODEL=waffle_pi \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    RCUTILS_LOGGING_USE_STDOUT=1

WORKDIR /opt/omokai_ws
RUN git clone --filter=blob:none --no-checkout \
      https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin.git \
      src/gazebo_ros_actor_plugin \
    && git -C src/gazebo_ros_actor_plugin checkout "${ACTOR_PLUGIN_REV}" \
    && rm -rf src/gazebo_ros_actor_plugin/.git
COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" \
      --skip-keys ament_python -r -y \
    && colcon build --merge-install --event-handlers console_direct+ \
    && rm -rf build log

COPY docker/entrypoint.sh /usr/local/bin/omokai-entrypoint
COPY docker/healthcheck.sh /usr/local/bin/omokai-healthcheck
RUN chmod 0755 /usr/local/bin/omokai-entrypoint /usr/local/bin/omokai-healthcheck \
    && chown -R omokai:omokai /opt/omokai_ws /home/omokai

USER omokai
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/omokai-entrypoint"]
CMD ["bash"]
