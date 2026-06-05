# Dockerfile — Allows to build a docker image
# A Container — is the executing image
# From one existing image one or more container
# If defines the commands to execute
# Once done it becomes our docker image

# ── My Image from the last commit ────────────────────────────────────────────────
# base immage (immutable)
FROM ros2_humble_backup

# ── WS build ─────────────────────────────────────────────────────────────────────
ARG WORKSPACE=/workspace

# ── Env variable ─────────────────────────────────────────────────────────────────
ENV ROS_DISTRO=humble \
    WORKSPACE=${WORKSPACE}     

# ── Automatic Source (valid for every user) ──────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /etc/bash.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /etc/bash.bashrc

# Foxglove + ROS2: Foxglove Bridge connects to the ROS2 topics and ROS2 uses DDS to publish them 
# so they must have the same domain of DDS 
# Source WS if built
RUN echo "if [ -f /root/tbot3_nav_monitor/install/setup.bash ]; then source /root/tbot3_nav_monitor/install/setup.bash; fi" >> ~/.bashrc

# ── Entrypoint for the source file ───────────────────────────────────────────────
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ── WS ───────────────────────────────────────────────────────────────────────────
WORKDIR ${WORKSPACE}

# ── FloxGlove Port ───────────────────────────────────────────────────────────────
EXPOSE 8765

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
# open shell bash when the container starts 
CMD ["bash"]