import rclpy

from vision.image_enhancement  import image_enhancement_utils
from vision.image_enhancement  import enhancement_algorithms as enhance

def main(args=None):
    rclpy.init(args=args)

    enhancer = enhance.ImageEnhancer(
        enhance.DCPEnhancement(),
        enhance.CLAHEEnhancement()
    )
    enhance_node = image_enhancement_utils.EnhanceNode(
        node_name="down_image_enhancement",
        input_topic="sensors/down_cam/image_raw",
        output_topic="vision/down_cam/image_enhanced",
        enhancer=enhancer
    )
    rclpy.spin(enhance_node)

    enhance_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
