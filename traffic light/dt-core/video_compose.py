import cv2
import os

# 设置参数
image_folder = "./1737955279.7"  # 图片目录
output_video = "output_video.mp4"  # 输出视频文件名
fps = 2  # 每秒帧数

# 获取图片列表并排序
images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
sorted_files = sorted(images, key=lambda x: float(x.split('_')[1].replace('.jpg', '')))
# key=lambda x: float(x.split('_')[1].replace('.jpg', ''))
# for img in images:
#     print(float((img.split('.')[1]).split('.')[0]))
# print(sorted_files)
#
# 获取图片尺寸
first_image = cv2.imread(os.path.join(image_folder, sorted_files[0]))
height, width, layers = first_image.shape

# 初始化视频写入器
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 格式
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# 写入每张图片
for image in sorted_files:
    img_path = os.path.join(image_folder, image)
    frame = cv2.imread(img_path)
    video.write(frame)

video.release()
print(f"视频保存为 {output_video}")


# echo "export ROS_MASTER_URI=http://192.168.10.13:11311" >> ~/.bashrc
# echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
# source ~/.bashrc
#
# export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311
# export ROS_IP=$(hostname -I | awk '{print $1}')
#
# export ROS_MASTER_URI=http://192.168.10.13:11311
# export ROS_IP=$(hostname -I | awk '{print $1}')


# docker ps -a --no-trunc
# IMAGE                                                        COMMAND                                                                                 CREATED        STATUS                         PORTS     NAMES
# duckietown/object-detection:latest-arm64v8                   "bash"                                                                                  3 weeks ago    Exited (0) 19 hours ago                  my-object-detection-test
# duckietown/dt-core:daffy-arm64v8                             "bash"                                                                                  3 weeks ago    Up About an hour (healthy)               my-dt-core-test
# duckietown/dt-gui-tools:daffy-arm32v7                        "/entrypoint.sh roslaunch virtual_joystick virtual_joystick_cli.launch veh:=xiao0o0o"   3 months ago   Exited (0) 3 months ago                  joystick_cli_xiao0o0o
# docker.io/duckietown/dt-car-interface:v4.1.0-arm64v8         "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               car-interface
# docker.io/duckietown/dt-device-dashboard:v4.1.0-arm64v8      "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (unhealthy)             dashboard
# docker.io/duckietown/dt-duckiebot-interface:v4.3.1-arm64v8   "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               duckiebot-interface
# docker.io/duckietown/dt-rosbridge-websocket:v4.1.0-arm64v8   "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               rosbridge-websocket
# docker.io/duckietown/dt-code-api:v4.1.0-arm64v8              "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               code-api
# docker.io/duckietown/dt-device-proxy:v4.2.0-arm64v8          "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               device-proxy
# docker.io/duckietown/dt-ros-commons:v4.3.0-arm64v8           "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               ros
# docker.io/duckietown/dt-files-api:v4.1.0-arm64v8             "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               files-api
# docker.io/duckietown/dt-device-online:v4.3.0-arm64v8         "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               device-online
# docker.io/duckietown/portainer:daffy-arm64v8                 "/portainer --host=unix:///var/run/docker.sock --no-auth"                               3 months ago   Up About an hour                         portainer
# docker.io/duckietown/dt-device-health:v4.2.1-arm64v8         "/entrypoint.sh bash -c dt-launcher-${DT_LAUNCHER}"                                     3 months ago   Up About an hour (healthy)               device-health




