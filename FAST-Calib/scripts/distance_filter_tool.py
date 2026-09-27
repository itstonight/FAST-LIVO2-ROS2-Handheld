#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：
1) 自动检测 rosbag2 中雷达点云类型：
   - sensor_msgs/msg/PointCloud2  (如 /hesai/pandar)
   - livox_ros_driver2/msg/CustomMsg (如 /livox/lidar)
2) 按各自的解析方式把点云导出成一个带 intensity 的 PCD 文件 (x y z intensity, ASCII)
3) 使用 Open3D 对该 PCD 进行交互选点（至少 4 个点），并根据 4 个点计算包围范围，
   保存为同名 .txt 文件。

依赖：
    - rosbag2_py
    - rclpy
    - sensor_msgs_py
    - rosidl_runtime_py
    - livox_ros_driver2 (for CustomMsg)
    - open3d, numpy

用法示例：
    python distance_filter_tool.py
    python distance_filter_tool.py /path/to/bag_dir /path/to/output_dir
    注意：ROS2 的 bag 是目录，不是 .bag 文件
"""

import os
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import open3d as o3d

# Try to import livox_ros_driver2 - if not available, we'll handle it
try:
    from livox_ros_driver2.msg import CustomMsg
    HAS_LIVOX = True
except ImportError:
    HAS_LIVOX = False
    print("[WARN] livox_ros_driver2 not available, CustomMsg support disabled", file=sys.stderr)

# ===================== 通用：保存 PCD =====================

def save_pcd_with_intensity(points, intensities, output_path):
    """
    保存点云为带 intensity 字段的 PCD 文件 (ASCII 格式)
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")

# ===================== 情况 1：PointCloud2 =====================

def find_intensity_field(msg):
    """在 PointCloud2 的 fields 中自动检测强度字段名称"""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_dir,
    output_dir,
    topic_name="/hesai/pandar",
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    将 rosbag2 中 PointCloud2 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    print(f"[Bag] 打开 rosbag2: {bag_dir}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] 无法打开 bag: {e}", file=sys.stderr)
        return None

    # 获取 topic 类型
    metadata = reader.get_metadata()
    topic_type_map = {}
    for topic_info in metadata.topics_with_message_count:
        topic_type_map[topic_info.topic_metadata.name] = topic_info.topic_metadata.type

    if topic_name not in topic_type_map:
        print(f"[ERROR] Topic '{topic_name}' 不存在于 bag 中", file=sys.stderr)
        # reader.close()  # SequentialReader 不支持 close() 方法
        return None

    topic_type = topic_type_map[topic_name]
    if topic_type != "sensor_msgs/msg/PointCloud2":
        print(f"[ERROR] Topic '{topic_name}' 类型不是 PointCloud2，而是 {topic_type}", file=sys.stderr)
        # reader.close()  # SequentialReader 不支持 close() 方法
        return None

    # 获取 PointCloud2 消息类
    PointCloud2Msg = get_message("sensor_msgs/msg/PointCloud2")

    # 1) 先检测强度字段（读取第一个消息）
    intensity_field = None
    first_msg = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            first_msg = deserialize_message(data, PointCloud2Msg)
            intensity_field = find_intensity_field(first_msg)
            if intensity_field:
                print(f"[Bag] 检测到 intensity 字段: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] 未找到强度字段! 退出 PointCloud2 转换。", file=sys.stderr)
        # reader.close()  # SequentialReader 不支持 close() 方法
        return None

    # 重新打开 reader 从头读取
    # reader.close()  # SequentialReader 不支持 close() 方法
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # 2) 读取指定 topic 的所有点云
    all_points = []
    all_intensities = []

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 PointCloud2 点云...")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue

        try:
            msg = deserialize_message(data, PointCloud2Msg)
            field_names = ["x", "y", "z", intensity_field]
            for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                all_points.append([point[0], point[1], point[2]])
                all_intensities.append(point[3])  # 强度是第四个字段
        except Exception as e:
            print(f"[ERROR] 读取错误: {str(e)}", file=sys.stderr)
            continue

    # reader.close()  # SequentialReader 不支持 close() 方法

    if not all_points:
        print("[ERROR] 未找到 PointCloud2 点云数据！", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path

# ===================== 情况 2：Livox CustomMsg =====================

def parse_livox_custom_msg(msg):
    """
    从 livox_ros_driver2/msg/CustomMsg 中解析 x, y, z, reflectivity
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities

def convert_livox_custom_bag_to_pcd(
    bag_dir,
    output_dir,
    topic_name="/livox/lidar",
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    将 rosbag2 中 livox_ros_driver2/msg/CustomMsg 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    if not HAS_LIVOX:
        print("[ERROR] livox_ros_driver2 不可用，无法处理 CustomMsg", file=sys.stderr)
        return None

    print(f"[Bag] 打开 rosbag2: {bag_dir}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] 无法打开 bag: {e}", file=sys.stderr)
        return None

    # 获取 CustomMsg 消息类
    CustomMsgMsg = get_message("livox_ros_driver2/msg/CustomMsg")

    all_points = []
    all_intensities = []

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 CustomMsg 点云...")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue

        try:
            msg = deserialize_message(data, CustomMsgMsg)
            pts, intens = parse_livox_custom_msg(msg)
            all_points.extend(pts)
            all_intensities.extend(intens)
        except Exception as e:
            print(f"[ERROR] 读取错误: {str(e)}", file=sys.stderr)
            continue

    # reader.close()  # SequentialReader 不支持 close() 方法

    if not all_points:
        print("[ERROR] 未找到 Livox CustomMsg 点云数据!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path

# ===================== 自动检测：这个 bag 用哪种方式 =====================

def detect_lidar_msg_type(bag_dir):
    """
    在 bag 里扫一圈，检测是否有 PointCloud2 或 Livox CustomMsg。
    返回：
        "PointCloud2", "CustomMsg", 或 None
    如果两种都有，默认优先 PointCloud2,并打印提示。
    """
    has_pc2 = False
    has_livox = False

    print(f"[Detect] 扫描 bag: {bag_dir}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERROR] 无法打开 bag: {e}", file=sys.stderr)
        return None

    metadata = reader.get_metadata()
    for topic_info in metadata.topics_with_message_count:
        topic_type = topic_info.topic_metadata.type
        if topic_type == "sensor_msgs/msg/PointCloud2":
            has_pc2 = True
        elif topic_type == "livox_ros_driver2/msg/CustomMsg":
            has_livox = True

    # reader.close()  # SequentialReader 不支持 close() 方法

    if has_pc2 and has_livox:
        print("[Detect] 同时检测到 PointCloud2 和 Livox CustomMsg, 默认使用 PointCloud2。")
        return "PointCloud2"
    elif has_pc2:
        print("[Detect] 检测到 PointCloud2 点云。")
        return "PointCloud2"
    elif has_livox:
        print("[Detect] 检测到 Livox CustomMsg 点云。")
        return "CustomMsg"
    else:
        print("[Detect] 未检测到 PointCloud2 或 Livox CustomMsg 点云。")
        return None

# ===================== Open3D 交互选点 & 保存范围 =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    在给定目录中读取指定 PCD 文件，用 Open3D 交互式选点并保存范围。
    """
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] 指定的 PCD 文件不存在: {pcd_path}", file=sys.stderr)
        return

    # 读取点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} 中没有点云数据，已跳过", file=sys.stderr)
        return

    print(f"\n正在处理: {target_pcd_name}")
    print("请在可视化窗口中按住 Shift 用鼠标左键选择点(至少4个)，然后按 Q 键关闭窗口")

    # 创建可视化窗口并添加点云
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"选择点 - {target_pcd_name}")
    vis.add_geometry(pcd)

    # 等待用户交互（Shift+左键选点, Q 退出）
    vis.run()
    vis.destroy_window()

    # 获取用户选择的点的索引
    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] 未选择任何点，{target_pcd_name} 没有保存文件", file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] 只选中了 {len(selected_indices)} 个点，少于 4 个，跳过该文件", file=sys.stderr)
        return

    # 只取前 4 个点
    selected_indices = selected_indices[:4]

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]  # 形状 (4, 3)

    # 计算四个点在各轴上的最小值和最大值
    mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
    maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

    # 按你的定义扩展 0.2m
    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    # 生成保存文件名 (与 PCD 文件同名，改为 txt)
    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] 已保存选点与范围到: {save_file}")
    print("点云处理完成。")

# ===================== main =====================

if __name__ == "__main__":
    # 1) 解析命令行参数：bag 路径 & 输出目录
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        # 默认使用当前目录下的某个 bag，可以按需修改
        bag_path = os.path.join(os.getcwd(), "my_bag")
        print(f"未指定 bag 目录，默认使用: {bag_path}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"未指定输出目录，使用当前目录: {output_dir}")

    # ROS2: bag 是目录，不是文件
    if not os.path.isdir(bag_path):
        print(f"[ERROR] bag 目录 '{bag_path}' 不存在", file=sys.stderr)
        print(f"[INFO] 提示: ROS2 的 bag 是目录（如 /path/to/my_bag），不是 .bag 文件", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] 输出目录 '{output_dir}' 不存在", file=sys.stderr)
        sys.exit(1)

    # 3) 自动检测 bag 中点云类型
    msg_type = detect_lidar_msg_type(bag_path)
    if msg_type is None:
        print("[ERROR] 未检测到支持的雷达消息类型，退出。", file=sys.stderr)
        sys.exit(1)

    # 4) 根据类型做对应的 PCD 转换
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_dir=bag_path,
            output_dir=output_dir,
            topic_name="/livox/lidar",  # 如有不同，可改成 topic 名称
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_dir=bag_path,
            output_dir=output_dir,
            topic_name="/livox/lidar",  # 如有不同，可改成 topic 名称
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD 生成失败，退出。", file=sys.stderr)
        sys.exit(1)

    # 5) 对刚生成的这个 PCD 做交互式选点 + 范围保存
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
