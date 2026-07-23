# ArduPilot SITL (ArduCopter) — cross-platform, pinned to a stable release.
# Builds on both amd64 and arm64 (Apple Silicon) natively.

FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates g++ pkg-config \
    python3 python3-pip python3-setuptools python3-dev \
    && rm -rf /var/lib/apt/lists/*

# empy must be < 4.0 for the ArduPilot build scripts
RUN pip3 install --no-cache-dir empy==3.3.4 pexpect future

ARG ARDUPILOT_TAG=Copter-4.5.7
RUN git clone --depth 1 --branch ${ARDUPILOT_TAG} \
    https://github.com/ArduPilot/ardupilot.git /ardupilot \
    && cd /ardupilot \
    && git submodule update --init --recursive --depth 1

WORKDIR /ardupilot
RUN ./waf configure --board sitl && ./waf copter

# ---- runtime image ----
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6 libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ardupilot/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY --from=builder /ardupilot/Tools/autotest/default_params/copter.parm /sitl/copter.parm

WORKDIR /sitl

# MAVLink TCP ports (5760 = primary, 5762/5763 = extra)
EXPOSE 5760 5762 5763

# -w wipes virtual EEPROM on start (container is ephemeral anyway)
# --speedup 1 must be explicit: ARM64 builds crash with a floating point
#   exception without it (https://github.com/ArduPilot/ardupilot/issues/19588)
# Home: CMAC field near Canberra, Australia — the standard SITL test location
ENTRYPOINT ["arducopter", "-w", "--model", "quad", \
    "--speedup", "1", \
    "--defaults", "/sitl/copter.parm", \
    "--home", "-35.363261,149.165230,584,353", \
    "-I0"]
