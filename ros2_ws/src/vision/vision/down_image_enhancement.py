import rclpy

from vision.image_enhancement import image_enhancement_utils
from vision.image_enhancement import enhancement_algorithms as enhance
from vision.image_enhancement import enhancement_algorithms_GPU as GPUenhance

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("down_image_enhancement")
    enhancer = enhance.CPUImageEnhancer(
        enhance.DCPEnhancement(),
        enhance.CLAHEEnhancement()
    )
    
    image_enhance = image_enhancement_utils.ImageEnhancement(
        node=node,
        enhancer=enhancer
    )
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
