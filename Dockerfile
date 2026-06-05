# Dockerfile — Allows to build a docker image
# A Container — is the executing image
# From one existing image one or more container
# If defines the commands to execute
# Once done it becomes our docker image

# ── My Image from the last commit ────────────────────────────────────────────────
# base immage (immutable)
FROM ros2_humble_backup

ARG WORKSPACE=/workspace

ENV ROS_DISTRO=humble \
    WORKSPACE=${WORKSPACE}     

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR ${WORKSPACE}
EXPOSE 8765

ENTRYPOINT ["bash", "/usr/local/bin/entrypoint.sh"]
CMD ["bash"]