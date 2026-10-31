# SpecsReach station image: the AUBO arm bridge and the Spectacles relay.
#
#   docker build -t specsreach-station station/
#
# Only what those two programs need: Python 3.12, GStreamer's WebRTC stack
# (webrtcbin + libnice) with its Python bindings, PyYAML, websockets and
# AUBO's SDK. The code is not copied in: the start scripts mount the repo, so
# a config or tuning edit only needs a restart, never a rebuild.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

# plugins-bad holds webrtcbin (DTLS, SCTP data channels), gstreamer1.0-nice
# the ICE transport, and gir1.2-gst-plugins-bad-1.0 the GstWebRTC Python
# bindings (GstSdp comes with gir1.2-gst-plugins-base-1.0).
RUN apt-get update && apt-get install -y --no-install-recommends \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-nice \
        gir1.2-gstreamer-1.0 \
        gir1.2-gst-plugins-base-1.0 \
        gir1.2-gst-plugins-bad-1.0 \
        python3 \
        python3-gi \
        python3-yaml \
        python3-pip \
        iproute2 \
        iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# pyaubo-sdk: AUBO's SDK for ARCS controllers (`arm_backend: aubo`). PyPI only
# has pre-release tags. The 0.26 line matches a controller whose pendant shows
# "SDK Interface Version 0.26.0" (About screen); re-pin for another version.
# websockets: the relay's server for the Lens; 13.1 is the version it was
# tested with. Ubuntu 24.04 marks its Python externally managed, hence
# --break-system-packages (this image runs nothing else).
RUN pip install --no-cache-dir --break-system-packages \
        "pyaubo-sdk==0.26.0rc6" \
        "websockets==13.1"

WORKDIR /workspace
CMD ["bash"]
