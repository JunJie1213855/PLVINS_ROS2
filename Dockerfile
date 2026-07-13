# ============================================================
# PL-VINS Docker 镜像
#
# 构建:
#   docker build -t plvins:humble .
#
# 运行:
#   docker compose up      # 带 RViz 界面
#   docker run -it --rm plvins:humble bash   # 纯命令行
# ============================================================

FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# 系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libceres-dev \
    libeigen3-dev \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# evo 评估工具
RUN pip3 install evo --upgrade

# 普通用户
RUN useradd -m -s /bin/bash ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ros
WORKDIR /home/ros/plvins_ws

# ROS2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc

# 源码
COPY --chown=ros:ros src/ src/

# 编译（先消息包，再全部）
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select plvins_interfaces && \
    source install/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /home/ros/plvins_ws/install/setup.bash" >> /home/ros/.bashrc

# 根目录文件
COPY --chown=ros:ros eval_euroc.sh docs/ README.md ./

# 入口
COPY --chown=ros:ros docker_entrypoint.sh /home/ros/
RUN chmod +x /home/ros/docker_entrypoint.sh
ENTRYPOINT ["/home/ros/docker_entrypoint.sh"]
CMD ["bash"]
