import rclpy

from vision.image_enhancement  import image_enhancement_utils
from vision.image_enhancement  import enhancement_algorithms as enhance
from vision.image_enhancement  import enhancement_algorithms_GPU as GPUenhance

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("front_image_enhancement")
    # add DSC model path parameter declaration and retrieval specifically for front cam
    node.declare_parameter("dsc_model_path", rclpy.Parameter.Type.STRING)
    
    dsc_model_path = node.get_parameter("dsc_model_path").get_parameter_value().string_value
    if not dsc_model_path:
        node.get_logger().error("DSC model path parameter 'dsc_model_path' is required but was not provided. Please set this parameter to the path of the DSC pipeline JIT model.")
        node.get_logger().fatal("Shutting down EnhanceNode due to missing DSC model path.")
        raise RuntimeError("DSC model path parameter 'dsc_model_path' is required but was not provided.")
    
    enhancer = GPUenhance.GPUImageEnhancer(GPUenhance.DeepSeeColor(dsc_model_path))
    image_enhance = image_enhancement_utils.ImageEnhancement(
        node=node,
        enhancer=enhancer
    )
    
    
    enhancer.add_algorithm(GPUenhance.DeepSeeColor(dsc_model_path))
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
