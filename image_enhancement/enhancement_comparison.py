#!/usr/bin/env python3
"""
AUV Image Enhancement Comparison Tool

This script compares various image enhancement algorithms commonly used for
underwater AUV camera feeds for initial experimentation.

Required dependencies:
opencv-python>=4.8.0
numpy>=1.21.0
matplotlib>=3.5.0
Pillow>=9.0.0
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from typing import List

import image_enhancement as ie

def create_comparison_grid(original: np.ndarray, enhanced_images: List[np.ndarray], 
                          titles: List[str], output_path: str, max_width: int = 1200):
    """Create a comparison grid showing original and enhanced images."""
    
    # Calculate grid dimensions
    n_images = len(enhanced_images) + 1  # +1 for original
    cols = min(4, n_images)  # Max 4 columns
    rows = (n_images + cols - 1) // cols  # Ceiling division
    
    # Resize images to fit grid
    target_height = 300
    target_width = int(target_height * original.shape[1] / original.shape[0])
    
    # Resize original
    original_resized = cv2.resize(original, (target_width, target_height))
    
    # Resize enhanced images
    enhanced_resized = []
    for img in enhanced_images:
        enhanced_resized.append(cv2.resize(img, (target_width, target_height)))
    
    # Create figure
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 4, rows * 3))
    if rows == 1:
        axes = axes.reshape(1, -1)
    elif cols == 1:
        axes = axes.reshape(-1, 1)
    
    # Plot original
    axes[0, 0].imshow(cv2.cvtColor(original_resized, cv2.COLOR_BGR2RGB))
    axes[0, 0].set_title('Original', fontsize=12, fontweight='bold')
    axes[0, 0].axis('off')
    
    # Plot enhanced images
    idx = 1
    for i in range(rows):
        for j in range(cols):
            if i == 0 and j == 0:
                continue  # Skip original position
            
            if idx <= len(enhanced_resized):
                axes[i, j].imshow(cv2.cvtColor(enhanced_resized[idx-1], cv2.COLOR_BGR2RGB))
                axes[i, j].set_title(titles[idx-1], fontsize=10)
                axes[i, j].axis('off')
                idx += 1
            else:
                axes[i, j].axis('off')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='AUV Image Enhancement Comparison Tool')
    parser.add_argument('--input', '-i', type=str, default='input', 
                       help='Input directory containing images (default: input)')
    parser.add_argument('--output', '-o', type=str, default='output', 
                       help='Output directory for comparison images (default: output)')
    parser.add_argument('--image', type=str, help='Specific image file to process')
    parser.add_argument('--max-width', type=int, default=1200, 
                       help='Maximum width for output images (default: 1200)')
    
    args = parser.parse_args()
    
    # Set up paths
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(exist_ok=True)
    
    # Get image files
    if args.image:
        image_files = [Path(args.image)]
    else:
        image_files = list(input_dir.glob('*.jpg')) + list(input_dir.glob('*.png')) + list(input_dir.glob('*.jpeg'))
    
    if not image_files:
        print(f"No images found in {input_dir}")
        return
    
    # Initialize enhancer
    enhancer = ie.ImageEnhancer(
        ie.UnderwaterColorCorrection(),
        ie.DCPEnhancement(),
        ie.CLAHEEnhancement()
    )

    # Define enhancement methods
    enhancement_methods = [
        ('CLAHE', ie.CLAHEEnhancement()),
        ('Gamma Correction (γ=1.5)', ie.GammaCorrection(1.5)),
        ('White Balance (Gray World)', ie.WhiteBalance('gray_world')),
        ('Red Enhancement', ie.RedChannelEnhancement(1.2)),
        ('Dark Channel Prior', ie.DCPEnhancement()),
        ('Histogram Equalization', ie.HistogramEqualization()),
        ('Unsharp Masking', ie.UnsharpMasking()),
        ('Underwater Color Correction', ie.UnderwaterColorCorrection()),
        ('Bilateral Filter', ie.BilateralFilter()),
        ('Combined Enhancement', enhancer)
    ]
    
    # Process each image
    for image_file in image_files:
        print(f"Processing {image_file.name}...")
        
        # Load image
        image = cv2.imread(str(image_file))
        if image is None:
            print(f"Error loading {image_file}")
            continue
        
        # Apply enhancements
        enhanced_images = []
        titles = []
        
        for title, method in enhancement_methods:
            try:
                enhanced = method(image)
                enhanced_images.append(enhanced)
                titles.append(title)
            except Exception as e:
                print(f"Error applying {title}: {e}")
        
        # Create comparison grid
        output_path = output_dir / f"comparison_{image_file.stem}.png"
        create_comparison_grid(image, enhanced_images, titles, str(output_path), args.max_width)
        
        print(f"Comparison saved to {output_path}")
    
    print("Processing complete!")

if __name__ == "__main__":
    main()
