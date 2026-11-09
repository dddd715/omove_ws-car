cat > ~/omove_ws/src/omove_description/urdf/omove.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot name="omove" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============ 物理参数（按PDF表3-1） ============ -->
  <xacro:property name="body_len" value="0.52"/>   <!-- 车身长 m -->
  <xacro:property name="body_wid" value="0.36"/>  <!-- 车身宽 m -->
  <xacro:property name="body_hgt" value="0.05"/>  <!-- 底盘板厚 m -->
  <xacro:property name="wheel_r"  value="0.076"/> <!-- 轮半径 152mm/2 -->
  <xacro:property name="wheel_w"  value="0.04"/>  <!-- 轮宽 m -->
  <xacro:property name="wheel_base_x" value="0.18"/> <!-- ½前后轮距 -->
  <xacro:property name="wheel_track_y" value="0.20"/><!-- ½左右轮距 -->
  <xacro:property name="chassis_z" value="0.11"/>  <!-- 底盘中心离地高度 -->

  <!-- 通用材质 -->
  <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
  <material name="blue"><color rgba="0.2 0.4 0.8 1"/></material>
  <material name="gray"><color rgba="0.6 0.6 0.6 1"/></material>

  <!-- ============ base_link 底盘 ============ -->
  <link name="base_link">
    <visual>
      <geometry><box size="${body_len} ${body_wid} ${body_hgt}"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><box size="${body_len} ${body_wid} ${body_hgt}"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- ============ 四个麦克纳姆轮 ============ -->
  <!-- 宏：单个轮子，side=+1左/-1右 -->
  <xacro:macro name="wheel" params="prefix x y">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${wheel_r - chassis_z}" rpy="${pi/2} 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
    <link name="${prefix}_wheel">
      <visual>
        <geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry>
        <origin rpy="${pi/2} 0 0"/>
        <material name="blue"/>
      </visual>
      <collision>
        <geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry>
        <origin rpy="${pi/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.004"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:wheel prefix="front_left"  x="${wheel_base_x}"  y="${wheel_track_y}"/>
  <xacro:wheel prefix="front_right" x="${wheel_base_x}"  y="${-wheel_track_y}"/>
  <xacro:wheel prefix="rear_left"   x="${-wheel_base_x}" y="${wheel_track_y}"/>
  <xacro:wheel prefix="rear_right"  x="${-wheel_base_x}" y="${-wheel_track_y}"/>

  <!-- ============ 虚拟激光雷达（替代思岚A1） ============ -->
  <link name="laser_link"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- 导入Gazebo插件配置 -->
  <xacro:include filename="$(find omove_description)/urdf/gazebo.urdf.xacro"/>
</robot>
EOF

