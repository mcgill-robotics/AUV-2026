# AUV Image Enhancement Comparison Tool

This tool provides a comprehensive comparison of various image enhancement algorithms specifically designed for underwater AUV camera feeds. It processes input images and creates visual comparisons showing the original image alongside multiple enhancement techniques.

## Features

The tool includes the following enhancement algorithms:

1. **CLAHE (Contrast Limited Adaptive Histogram Equalization)** - Improves local contrast
2. **Gamma Correction** - Adjusts brightness and contrast using power-law transformation
3. **White Balance** - Corrects color temperature using Gray World and White Patch methods
4. **Red Channel Enhancement** - Counteracts underwater blue/green dominance
5. **Dark Channel Prior (DCP)** - Dehazing algorithm for underwater visibility
6. **Histogram Equalization** - Improves global contrast
7. **Unsharp Masking** - Sharpens images while preserving edges
8. **Underwater Color Correction** - Specialized color correction for underwater images
9. **Bilateral Filter** - Noise reduction while preserving edges

## Installation

1. Install the required dependencies:
```bash
pip install opencv-python>=4.8.0 numpy>=1.21.0 matplotlib>=3.5.0 Pillow>=9.0.0
```

Or install all at once:
```bash
pip install opencv-python numpy matplotlib Pillow
```

## Usage

### Basic Usage

1. Place your underwater camera feed images in the `input/` directory
2. Run the enhancement comparison script:
```bash
python enhancement_comparison.py
```

### Advanced Usage

```bash
# Process a specific image
python enhancement_comparison.py --image path/to/your/image.jpg

# Specify custom input/output directories
python enhancement_comparison.py --input /path/to/input --output /path/to/output

# Adjust output image size
python enhancement_comparison.py --max-width 1600
```

### Command Line Options

- `--input, -i`: Input directory containing images (default: `input`)
- `--output, -o`: Output directory for comparison images (default: `output`)
- `--image`: Process a specific image file
- `--max-width`: Maximum width for output images (default: 1200)

## Directory Structure

```
image_enhancement/
├── image_enhancement.py    # Contain enhancement algorithms
├── enhancement_comparison.py # Generates comparisons between algorithms
├── README.md                   # This file
├── input/                      # Place your images here (gitignored)
│   └── *.jpg, *.png, *.jpeg   # Input images
└── output/                     # Comparison results (gitignored)
    └── comparison_*.png        # Generated comparison images
```

**Note**: The `input/` and `output/` directories are automatically ignored by git to prevent large image files from being committed to the repository. This is configured in the main project's `.gitignore` file.

## Input Requirements

- Supported formats: `.jpg`, `.png`, `.jpeg`
- Images should be underwater camera feeds for best results
- The tool will process all images in the input directory by default

## Output

The tool generates comparison grids showing:
- Original image
- All enhancement algorithms applied to the same image
- Side-by-side comparison for easy visual evaluation

Output files are saved as `comparison_[original_filename].png` in the output directory.

## Algorithm Details

### CLAHE
- **Purpose**: Improves local contrast without over-amplifying noise
- **Best for**: Images with varying lighting conditions
- **Parameters**: Clip limit (2.0), tile grid size (8x8)

### Gamma Correction
- **Purpose**: Adjusts brightness and contrast using power-law transformation
- **Best for**: Dark or overexposed images
- **Parameters**: Gamma value (1.5 for underwater images)

### White Balance
- **Purpose**: Corrects color temperature and removes color casts
- **Best for**: Images with incorrect color temperature
- **Methods**: Gray World, White Patch

### Red Channel Enhancement
- **Purpose**: Counteracts the blue/green dominance in underwater images
- **Best for**: Images with strong blue/green color cast
- **Parameters**: Enhancement factor (1.2)

### Dark Channel Prior (DCP)
- **Purpose**: Dehazing algorithm that improves visibility in hazy conditions
- **Best for**: Murky or hazy underwater images
- **Parameters**: Window size (15), omega (0.95), t0 (0.1)

### Histogram Equalization
- **Purpose**: Improves global contrast by spreading out intensity values
- **Best for**: Images with poor contrast
- **Applied to**: Y channel in YUV color space

### Unsharp Masking
- **Purpose**: Sharpens images while preserving edges
- **Best for**: Blurry or soft images
- **Parameters**: Kernel size (5x5), sigma (1.0), amount (1.0)

### Underwater Color Correction
- **Purpose**: Specialized color correction matrix for underwater conditions
- **Best for**: General underwater color correction
- **Method**: Custom correction matrix

### Bilateral Filter
- **Purpose**: Noise reduction while preserving edges
- **Best for**: Noisy images
- **Parameters**: d (9), sigma_color (75), sigma_space (75)

## Tips for Best Results

1. **Start with one image**: Test with a single representative image first
2. **Compare visually**: Look for the enhancement that best improves visibility and color accuracy
3. **Consider your use case**: Different algorithms work better for different underwater conditions
4. **Combine techniques**: You can apply multiple enhancements in sequence for optimal results

## Troubleshooting

- **Import errors**: Make sure all dependencies are installed with `pip install opencv-python numpy matplotlib Pillow`
- **No images found**: Check that images are in the correct format (.jpg, .png, .jpeg) and in the input directory
- **Memory issues**: Reduce image size or process images one at a time
- **Poor results**: Try adjusting algorithm parameters in the script for your specific images

## Contributing

To add new enhancement algorithms:
1. In [image_enhancement.py](./image_enhancement.py), add a new class that inherits from `EnhancementAlgorithm`
2. Implement the `apply_algorithm` method, which only takes an numpy array image. Set optional parameters in the `__init__`
4. (Optional) In [enhancement_comparison.py](./enhancement_comparison.py) Add the method to the `enhancement_methods` list in the `main()` function
5. Update this README with algorithm details

To compose multiple algorithms, use the `ImageEnhancer` class:
 
- The constructor expects `EnhancementAlgorithm` objects
- The `__call__`, as with all algorithms, expects a numpy array image.
- The `to_torch_transform` method can be used to obtain the [PyTorch vision tranform](https://docs.pytorch.org/vision/stable/transforms.html) equivalent of the enhancer itself. This can easily be slotted into the YOLO object detection pipeline as a preprocessing step, for instance.

### Next Steps

Using a pytorch transform, while simple, is very slow, as it requires at least 2 conversions (Tensor $\to$ Numpy Array $\to$ Tensor), 4 including transfers to and from GPUs, and 6 including ROS topic image stream subscription and publishing. Thus a more feasible solution is to directly use ROS, as have a ROS package that:
1. Subscribes to the ZED Camera stream, which publishes a [Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) message
2. Converts the image to a BGR Numpy Array using 
```python
from cv_bridge import CvBridge
cv2_image = bridge.imgmsg_to_cv2(raw_image, "bgr8")
```
3. Applies desired enhancements using `ImageEnhancer`
4. Convert result back into a ROS Image message
```python
bridge.cv2_to_imgmsg(enhanced_img, "bgr8")
```
5. Publish the message to a ROS topic e.g. `/image_enhancement/`
6. This can then be picked up by the object detection pipeline

(instead of 4-6 we can add the `ImageEnhancer` directly in the object detection node to save on conversions)

See [here](https://github.com/mcgill-robotics/AUV-2025/blob/d258d7c2819ee4e45ed785ea26cf59be19d9b2ea/catkin_ws/src/vision/src/object_detection.py#L194) for a more involved example of implementation. While this still requires 2 conversions, there are several advantages:
- it is substantially faster than using a Pytorch transform
- it avoids requiring a complete PyTorch dependency in the project overall (the ultralytics handles a light dependency instead). The imports can then be removed from [image_enhancement.py](./image_enhancement.py) entirely
- it makes the image enhancement decoupled from the object detection, which makes refactoring easier 
- a ROS package makes it possible to optimize even further. Most of the algorithms, since they involve the OpenCV module, are also available in C++, and the enhancement can even be done on GPU with `cv::cuda`. Since online inference on an image stream should be done on GPU anyway, this would significantly reduce runtime of enhancement

The main disadvantage is that if image enhancement on a ROS node, then training the YOLO model on the enhanced images requires stores them on disk. 

## License

This tool is part of the AUV-2026 project by McGill Robotics.
