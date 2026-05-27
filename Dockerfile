# Dockerfile — Allows to build a docker image
# A Container — is the executing image
# From one existing image one or more container
# If defines the commands to execute
# Once done it becomes our docker image

# ── My Image from the last commit ────────────────────────────────────────────────
FROM ros2_humble_backup

# ── WS build ─────────────────────────────────────────────────────────────────────
ARG WORKSPACE = /tbot3_nav_monitor

# ── Env variable ─────────────────────────────────────────────────────────────────
ENV ROS_DISTRO = humble \
    WORKSPACE = ${WORKSPACE}     

# ── Automatic Source ─────────────────────────────────────────────────────────────
RUN echo " source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "if [-f &{WORKSPACE}/install/setup.bash ]; then" >> /etc/bash.bashrc && \
    echo " source &{WORKSPACE}/install/setup.bash" >> /etc/bash/bashrc && \
    echo "fi" >> /etc/bash.bashrc

# ── WS ───────────────────────────────────────────────────────────────────────────
WORKDIR ${WORKSPACE}

# ── FloxGlove Port ───────────────────────────────────────────────────────────────
EXPOSE 8765

CMD ["bash"]