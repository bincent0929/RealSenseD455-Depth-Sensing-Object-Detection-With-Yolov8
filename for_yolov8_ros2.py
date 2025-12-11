import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# AI CREATED CODE - NOT TESTED!

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/detected_object/pose', 10)
        self.info_pub = self.create_publisher(String, '/detected_object/info', 10)
        self.image_pub = self.create_publisher(Image, '/detected_object/image', 10)
        
        self.bridge = CvBridge()
        
        # Load the YOLOv8 model
        self.model = YOLO('yolo11s.pt')
        
        # Set up the RealSense D455 camera
        self.pipeline = rs.pipeline()
        config = rs.config()
        self.IMAGE_WIDTH = 640
        config.enable_stream(rs.stream.color, self.IMAGE_WIDTH, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.IMAGE_WIDTH, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        
        # Set the depth scale
        self.depth_scale = 0.0010000000474974513
        
        # Camera parameters for heading calculation
        self.IMAGE_CENTER_X = self.IMAGE_WIDTH // 2
        self.HORIZONTAL_FOV = 87  # degrees for RealSense D455
        
        # Create spatial and temporal filters for depth image
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        
        # Timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 FPS
        
        self.get_logger().info('Object Detection Node Started')
    
    def process_frame(self):
        try:
            # Get the latest frame from the camera
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # Convert the frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Apply filters to depth image
            depth_frame = self.spatial.process(depth_frame)
            depth_frame = self.temporal.process(depth_frame)
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Convert the depth image to meters
            depth_image = depth_image * self.depth_scale
            
            # Detect objects using YOLOv8
            results = self.model(color_image, verbose=False)
            
            # Process the results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf[0].cpu().numpy()
                    class_id = box.cls[0].cpu().numpy()
                    
                    if confidence < 0.5:
                        continue  # Skip detections with low confidence
                    
                    # Calculate the center of the bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Calculate heading (angle from camera center)
                    pixel_offset = center_x - self.IMAGE_CENTER_X
                    heading_angle = (pixel_offset / self.IMAGE_CENTER_X) * (self.HORIZONTAL_FOV / 2)
                    
                    # Calculate the distance to the object
                    object_depth = np.median(depth_image[y1:y2, x1:x2])
                    
                    # Publish pose message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera_link'
                    pose_msg.pose.position.x = float(object_depth)  # Distance forward
                    pose_msg.pose.position.y = float(-object_depth * np.tan(np.radians(heading_angle)))  # Lateral offset
                    pose_msg.pose.position.z = 0.0
                    self.target_pub.publish(pose_msg)
                    
                    # Publish info message
                    info_msg = String()
                    info_msg.data = f"{self.model.names[int(class_id)]}: {object_depth:.2f}m at {heading_angle:.1f}°"
                    self.info_pub.publish(info_msg)
                    
                    # Draw visualization
                    label = f"{object_depth:.2f}m, {heading_angle:.1f}°"
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (252, 119, 30), 2)
                    cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), -1)
                    cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)
                    
                    self.get_logger().info(f"{self.model.names[int(class_id)]}: {object_depth:.2f}m at {heading_angle:.1f}°")
            
            # Publish annotated image
            image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            self.image_pub.publish(image_msg)
            
            # Show the image
            cv2.imshow("Color Image", color_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
    
    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
