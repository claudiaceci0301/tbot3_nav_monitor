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