# realsense_sim_driver

这是一个 ROS 2 Python 包，用于接收 Isaac Sim 发布的 RGB/depth 图像和相机内参，将 depth 图像对齐到 RGB 相机坐标系，并发布 RealSense 兼容的话题，包括 color、depth、aligned depth、camera info 和带颜色的 PointCloud2 点云。

默认参数文件位于 `config/config.yaml`。

构建:

```bash
colcon build --packages-select realsense_sim_driver
source install/setup.bash
```

使用默认配置运行:

```bash
ros2 launch realsense_sim_driver realsense_sim_driver.launch.py
```

使用自定义配置文件运行:

```bash
ros2 launch realsense_sim_driver realsense_sim_driver.launch.py \
  config_file:=/absolute/path/to/config.yaml
```

默认 Isaac Sim 输入话题:

- `/isaac_rgb`
- `/isaac_depth`
- `/isaac_rgb_cam_info`
- `/isaac_depth_cam_info`

RealSense 兼容输出话题:

- `/color/image_raw`
- `/color/camera_info`
- `/depth/image_rect_raw`
- `/depth/camera_info`
- `/aligned_depth_to_color/image_raw`
- `/aligned_depth_to_color/camera_info`
- `/depth/color/points`

QoS 配置:

默认情况下，节点订阅 Isaac Sim 输入话题使用 `best_effort`，发布给 RViz 和下游节点的话题使用 `reliable`。这样可以兼容仿真传感器数据，同时避免 RViz 出现 `RELIABILITY_QOS_POLICY` 不兼容 warning。

相关参数位于 `config/config.yaml`:

```yaml
input_qos_reliability: best_effort
output_qos_reliability: reliable
qos_depth: 10
```

深度格式转换:

Isaac Sim 输入 depth 话题按 `32FC1` 读取，单位为米。默认配置将对齐后的 depth 输出为 `16UC1`，单位为毫米:

```text
depth_mm_uint16 = round(depth_m / uint16_depth_scale)
                = round(depth_m / 0.001)
```

例如 `1.25 m` 会输出为 `1250`。无效深度或未投影到 RGB 图像的像素输出为 `0`。

深度对齐数学原理:

节点会把每一个有效的 depth 像素先反投影成 depth 相机光学坐标系下的 3D 点，再通过外参变换到 RGB 相机光学坐标系，最后投影回 RGB 图像平面。

对于 depth 图像中的像素 `(u_d, v_d)`，其深度值为 `z_d`，单位为米。depth 相机内参为 `fx_d`, `fy_d`, `cx_d`, `cy_d`，则反投影为:

```text
X_d = z_d * inv(K_d) * [u_d, v_d, 1]^T
    = [
        (u_d - cx_d) * z_d / fx_d,
        (v_d - cy_d) * z_d / fy_d,
        z_d
      ]^T
```

如果 RGB 和 depth 的 optical frame 不同，节点会从 TF 获取 depth frame 到 RGB frame 的刚体变换:

```text
X_rgb = R_rgb_depth * X_d + t_rgb_depth
```

其中 `R_rgb_depth` 是旋转矩阵，`t_rgb_depth` 是平移向量。若两个相机共光心或启用了 `assume_identity_tf`，则等价于 `R = I`, `t = 0`。

得到 RGB 相机坐标系下的 3D 点后，使用 RGB 相机内参 `fx_rgb`, `fy_rgb`, `cx_rgb`, `cy_rgb` 投影到 RGB 图像平面:

```text
u_rgb = fx_rgb * X_rgb / Z_rgb + cx_rgb
v_rgb = fy_rgb * Y_rgb / Z_rgb + cy_rgb
```

投影结果会四舍五入到最近的 RGB 像素。超出 RGB 图像范围、在相机后方、或不在 `min_depth_m` 到 `max_depth_m` 范围内的点会被丢弃。如果多个 depth 像素投影到同一个 RGB 像素，节点使用 z-buffer 只保留最近的 `Z_rgb`，因此输出的 aligned depth 表示 RGB 相机光学坐标系下的深度。

对齐后的 depth 图像尺寸与 RGB 图像一致，使用 RGB 相机内参。点云则基于 aligned depth 再按 RGB 内参反投影生成，并从同位置 RGB 像素取颜色。

深度单位由配置文件控制: 输入 `32FC1` 使用 `float_depth_scale` 转换到米；输出 `16UC1` 使用 `uint16_depth_scale` 将米转换为毫米整数。

如果 RGB 和 depth 的 optical frame 不同，需要发布 depth 相机 frame 到 RGB 相机 frame 的 TF。默认情况下，节点在找不到 TF 时会回退到单位变换，因此共光心的仿真相机可以不额外发布 TF。
