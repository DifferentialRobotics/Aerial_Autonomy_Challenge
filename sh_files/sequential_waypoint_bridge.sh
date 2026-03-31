#!/bin/zsh
# sequential_waypoint_bridge.zsh
# 用法: zsh sequential_waypoint_bridge.zsh [drone_id] [default_z] [reach_distance] [depart_distance]

DRONE_ID=${1:-0}
DEFAULT_Z=${2:-0.0}
REACH_DIST=${3:-0.3}
DEPART_DIST=${4:-0.2}

# 无人机初始位置（与 launch 文件保持一致）
INIT_X=0.0
INIT_Y=0.0
INIT_Z=1.0

TMP_SCRIPT=$(mktemp /tmp/waypoint_bridge_XXXX.py)

cat > "$TMP_SCRIPT" << 'EOF'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from threading import Lock
import os


class SequentialWaypointBridge:
  def __init__(self):
    rospy.init_node('sequential_waypoint_bridge', anonymous=True)

    # 参数配置
    self.drone_id    = int(os.environ.get('BRIDGE_DRONE_ID', 0))
    self.default_z   = float(os.environ.get('BRIDGE_DEFAULT_Z', 1.0))
    self.reach_dist  = float(os.environ.get('BRIDGE_REACH_DIST', 0.4))
    self.depart_dist = float(os.environ.get('BRIDGE_DEPART_DIST', 0.5))

    # 无人机初始位置
    self.init_x = float(os.environ.get('BRIDGE_INIT_X', 0.0))
    self.init_y = float(os.environ.get('BRIDGE_INIT_Y', 0.0))
    self.init_z = float(os.environ.get('BRIDGE_INIT_Z', 1.0))

    # 内部状态
    self.waypoints       = []
    self.current_index   = -1
    self.current_pos     = None
    self.waiting_arrival = False
    self.departed        = False
    self.depart_pos      = None
    self.returning_home  = False   # 是否正在返回原点
    self.lock            = Lock()
    self.last_fsm_state  = ""

    # 发布目标点
    self.goal_pub = rospy.Publisher(
      '/goal',
      PoseStamped,
      queue_size=10
    )

    # 订阅 Unity 发来的目标点列表
    rospy.Subscriber('/publish_point', PoseArray, self.pose_array_callback)

    # 订阅里程计
    odom_topic = f'quad_0/lidar_slam/odom'
    rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

    # 订阅 FSM 状态（备用）
    fsm_topic = f'/drone_{self.drone_id}_planning/fsm_state'
    rospy.Subscriber(fsm_topic, String, self.fsm_callback)

    rospy.loginfo("[WaypointBridge] 启动成功")
    rospy.loginfo(f"  drone_id     = {self.drone_id}")
    rospy.loginfo(f"  default_z    = {self.default_z}")
    rospy.loginfo(f"  reach_dist   = {self.reach_dist}")
    rospy.loginfo(f"  depart_dist  = {self.depart_dist}")
    rospy.loginfo(f"  home         = ({self.init_x}, {self.init_y}, {self.init_z})")
    rospy.loginfo(f"  等待 /publish_point 话题...")
    rospy.spin()

  # -------------------------------------------------------
  # 收到目标点列表：先返回原点，再开始任务
  # -------------------------------------------------------
  def pose_array_callback(self, msg):
    point_num = len(msg.poses)
    if point_num == 0:
      rospy.logwarn("[WaypointBridge] 收到空的 PoseArray，忽略。")
      return

    with self.lock:
      self.waypoints       = []
      self.current_index   = -1
      self.waiting_arrival = False
      self.departed        = False
      self.depart_pos      = None
      self.returning_home  = False

      for pose in msg.poses:
        z = pose.position.z if pose.position.z > 0.1 else self.default_z
        self.waypoints.append((pose.position.x, pose.position.y, z))

    rospy.loginfo(f"[WaypointBridge] 收到 {point_num} 个目标点：")
    for i, (x, y, z) in enumerate(self.waypoints):
      rospy.loginfo(f"  waypoint[{i}]: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    # 检查当前是否已经在原点附近
    if self.current_pos is not None:
      dist_to_home = math.sqrt(
        (self.current_pos.x - self.init_x) ** 2 +
        (self.current_pos.y - self.init_y) ** 2 +
        (self.current_pos.z - self.init_z) ** 2
      )
      if dist_to_home > self.reach_dist:
        # 不在原点，先飞回去
        rospy.loginfo(
          f"[WaypointBridge] 无人机不在原点 (dist={dist_to_home:.3f}m)，"
          f"先返回原点 ({self.init_x}, {self.init_y}, {self.init_z})..."
        )
        self.go_home()
        return

    # 已在原点，直接开始任务
    rospy.loginfo("[WaypointBridge] 无人机已在原点，直接开始任务。")
    self.send_next_waypoint()

  # -------------------------------------------------------
  # 发送返回原点指令
  # -------------------------------------------------------
  def go_home(self):
    with self.lock:
      self.returning_home  = True
      self.waiting_arrival = True
      self.departed        = False
      self.depart_pos      = (
        self.current_pos.x,
        self.current_pos.y,
        self.current_pos.z
      ) if self.current_pos else None

    goal = PoseStamped()
    goal.header          = Header()
    goal.header.stamp    = rospy.Time.now()
    goal.header.frame_id = 'world'
    goal.pose.position.x = self.init_x
    goal.pose.position.y = self.init_y
    goal.pose.position.z = self.init_z
    goal.pose.orientation.w = 1.0

    self.goal_pub.publish(goal)
    rospy.loginfo(
      f"[WaypointBridge] 返回原点指令已发送: "
      f"({self.init_x}, {self.init_y}, {self.init_z})"
    )

  # -------------------------------------------------------
  # FSM 状态回调（备用判断）
  # -------------------------------------------------------
  def fsm_callback(self, msg):
    state = msg.data.strip()
    triggered = False

    with self.lock:
      prev = self.last_fsm_state
      self.last_fsm_state = state
      if self.waiting_arrival and prev == "EXEC_TRAJ" and state == "WAIT_TARGET":
        self.waiting_arrival = False
        self.departed        = False
        triggered = True
        returning = self.returning_home

    if triggered:
      if returning:
        rospy.loginfo("[WaypointBridge] FSM 触发：已返回原点，开始执行任务。")
        with self.lock:
          self.returning_home = False
        rospy.sleep(0.5)
        self.send_next_waypoint()
      else:
        rospy.loginfo(
          f"[WaypointBridge] FSM 触发到达第 "
          f"{self.current_index + 1}/{len(self.waypoints)} 个点"
        )
        rospy.sleep(0.3)
        self.send_next_waypoint()

  # -------------------------------------------------------
  # 里程计回调（主要判断）
  # -------------------------------------------------------
  def odom_callback(self, msg):
    self.current_pos = msg.pose.pose.position

    with self.lock:
      if not self.waiting_arrival or self.current_index < 0 and not self.returning_home:
        return
      if self.depart_pos is None:
        return
      if self.returning_home:
        target = (self.init_x, self.init_y, self.init_z)
      else:
        target = self.waypoints[self.current_index]
      depart_pos = self.depart_pos
      departed   = self.departed

    # 第一步：判断是否已经离开出发点
    if not departed:
      dist_from_depart = math.sqrt(
        (self.current_pos.x - depart_pos[0]) ** 2 +
        (self.current_pos.y - depart_pos[1]) ** 2 +
        (self.current_pos.z - depart_pos[2]) ** 2
      )
      if dist_from_depart < self.depart_dist:
        return
      else:
        with self.lock:
          self.departed = True
        rospy.loginfo(
          f"[WaypointBridge] 无人机已出发，开始监测到达状态... "
          f"(离出发点={dist_from_depart:.3f}m)"
        )

    # 第二步：判断是否到达目标点
    dist_to_target = math.sqrt(
      (self.current_pos.x - target[0]) ** 2 +
      (self.current_pos.y - target[1]) ** 2 +
      (self.current_pos.z - target[2]) ** 2
    )

    if dist_to_target < self.reach_dist:
      with self.lock:
        if not self.waiting_arrival:
          return
        self.waiting_arrival = False
        self.departed        = False
        returning = self.returning_home
        if returning:
          self.returning_home = False

      if returning:
        rospy.loginfo(
          f"[WaypointBridge] odom 触发：已返回原点 (dist={dist_to_target:.3f}m)，"
          f"开始执行任务。"
        )
        rospy.sleep(0.5)
        self.send_next_waypoint()
      else:
        rospy.loginfo(
          f"[WaypointBridge] odom 触发到达第 "
          f"{self.current_index + 1}/{len(self.waypoints)} 个点 "
          f"(dist={dist_to_target:.3f}m)"
        )
        rospy.sleep(0.3)
        self.send_next_waypoint()

  # -------------------------------------------------------
  # 发送下一个目标点
  # -------------------------------------------------------
  def send_next_waypoint(self):
    with self.lock:
      next_index = self.current_index + 1
      if next_index >= len(self.waypoints):
        rospy.loginfo("[WaypointBridge] 所有目标点完成！任务结束。")
        return
      self.current_index   = next_index
      self.waiting_arrival = True
      self.departed        = False
      self.depart_pos = (
        self.current_pos.x,
        self.current_pos.y,
        self.current_pos.z
      ) if self.current_pos else None
      x, y, z = self.waypoints[self.current_index]

    goal = PoseStamped()
    goal.header          = Header()
    goal.header.stamp    = rospy.Time.now()
    goal.header.frame_id = 'world'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.w = 1.0

    self.goal_pub.publish(goal)
    rospy.loginfo(
      f"[WaypointBridge] 发送第 {self.current_index + 1}/{len(self.waypoints)} 个点: "
      f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
    )


if __name__ == '__main__':
  try:
    SequentialWaypointBridge()
  except rospy.ROSInterruptException:
    pass
EOF

# 通过环境变量传参
export BRIDGE_DRONE_ID=$DRONE_ID
export BRIDGE_DEFAULT_Z=$DEFAULT_Z
export BRIDGE_REACH_DIST=$REACH_DIST
export BRIDGE_DEPART_DIST=$DEPART_DIST
export BRIDGE_INIT_X=$INIT_X
export BRIDGE_INIT_Y=$INIT_Y
export BRIDGE_INIT_Z=$INIT_Z

echo "=========================================="
echo " Sequential Waypoint Bridge"
echo " drone_id     = $DRONE_ID"
echo " default_z    = $DEFAULT_Z"
echo " reach_dist   = $REACH_DIST"
echo " depart_dist  = $DEPART_DIST"
echo " home         = ($INIT_X, $INIT_Y, $INIT_Z)"
echo "=========================================="

python3 "$TMP_SCRIPT"
rm -f "$TMP_SCRIPT"
