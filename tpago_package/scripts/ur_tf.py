import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

def depth_to_base_callback(msg):
    # 创建tf缓冲区和监听器
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # 获取从camera_color_frame到base_link的变换
        transform = tf_buffer.lookup_transform('base', 'camera_link', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform lookup failed: %s" % e)
        return

    # 遍历接收到的深度数据
    for i in range(0, len(msg.data), 3):
        midpoint_x = msg.data[i]
        midpoint_y = msg.data[i + 1]
        depth_z = msg.data[i + 2]

        # 创建PointStamped用于变换
        point_in_camera_frame = PointStamped()
        point_in_camera_frame.header.frame_id = 'camera_link'
        point_in_camera_frame.point.x = midpoint_x
        point_in_camera_frame.point.y = midpoint_y
        point_in_camera_frame.point.z = depth_z

        # 执行坐标变换
        try:
            point_in_base_frame = tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)
            # 输出转换后的坐标
            rospy.loginfo("Transformed point in base frame: x=%.2f, y=%.2f, z=%.2f" % 
                          (point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z))
        except tf2_ros.TransformException as e:
            rospy.logwarn("Transformation failed: %s" % e)
            continue

def main():
    rospy.init_node('depth_to_base_frame_node')
    rospy.Subscriber("/object_bbox", Float32MultiArray, depth_to_base_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
