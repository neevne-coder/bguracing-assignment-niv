FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pandas \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY publisher_node.py .

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 publisher_node.py"]