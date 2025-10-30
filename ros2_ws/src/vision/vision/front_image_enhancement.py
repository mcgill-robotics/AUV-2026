import rclpy

from ros2_ws.src.vision.vision.image_enhancement import image_enhancement_utils
from ros2_ws.src.vision.vision.image_enhancement import enhancement_algorithms as enhance

def main(args=None):
    rclpy.init(args=args)

    enhancer = enhance.ImageEnhancer(
        enhance.UnderwaterColorCorrection(),
        enhance.DCPEnhancement(),
        enhance.BilateralFilter(),
        enhance.CLAHEEnhancement()
    )
    enhance_node = image_enhancement_utils.EnhanceNode(
        node_name="front_image_enhancement",
        input_topic="sensors/zed2i/zed_node/stereo/image_rect_color",
        output_topic="vision/front_cam/image_enhanced",
        enhancer=enhancer
    )
    rclpy.spin(enhance_node)

    enhance_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
