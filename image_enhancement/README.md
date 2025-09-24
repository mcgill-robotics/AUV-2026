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
pip install -r requirements.txt
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
├── enhancement_comparison.py    # Main script
├── requirements.txt             # Python dependencies
├── README.md                   # This file
├── input/                      # Place your images here (gitignored)
└── output/                     # Comparison results (gitignored)
```

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

- **Import errors**: Make sure all dependencies are installed with `pip install -r requirements.txt`
- **No images found**: Check that images are in the correct format (.jpg, .png, .jpeg) and in the input directory
- **Memory issues**: Reduce image size or process images one at a time
- **Poor results**: Try adjusting algorithm parameters in the script for your specific images

## Contributing

To add new enhancement algorithms:
1. Add a new method to the `ImageEnhancer` class
2. Add the method to the `enhancement_methods` list in the `main()` function
3. Update this README with algorithm details

## License

This tool is part of the AUV-2026 project by McGill Robotics.
