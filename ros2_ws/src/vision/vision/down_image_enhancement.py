import rclpy

from vision.image_enhancement import image_enhancement_utils
from vision.image_enhancement import enhancement_algorithms as enhance
from vision.image_enhancement import enhancement_algorithms_GPU as GPUenhance

def main(args=None):
    rclpy.init(args=args)

    enhancer = enhance.CPUImageEnhancer(
        enhance.DCPEnhancement(),
        enhance.CLAHEEnhancement()
    )
    
    enhance_node = image_enhancement_utils.EnhanceNode(
        node_name="down_image_enhancement",
        enhancer=enhancer
    )
    rclpy.spin(enhance_node)

    enhance_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
