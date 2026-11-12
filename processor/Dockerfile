FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

ENV MPLBACKEND=Agg
WORKDIR /app
COPY track_analysis.py processor_node.py ./

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 processor_node.py"]