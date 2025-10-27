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
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from typing import List
import os
from skimage.metrics import structural_similarity
from tqdm import tqdm
import time
import warnings

import image_enhancement as ie

def test_ssim(original: np.ndarray, enhanced: np.ndarray) -> float:
    ssim_value, _ = structural_similarity(original, enhanced, full=True,channel_axis=2,data_range=enhanced.max() - enhanced.min())
    return ssim_value

def test_psnr(original: np.ndarray, enhanced: np.ndarray) -> float:
    mse = np.mean((original - enhanced) ** 2)
    if mse == 0:
        return 100
    max_pixel = 255.0
    psnr_value = 20 * np.log10(max_pixel / np.sqrt(mse))
    return psnr_value

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

def tabulate_metrics(filename: Path) -> None:
    """Tabulate metrics from a text file."""
    metrics = pd.read_csv(filename)
    print(f"===={filename.name}====")
    print(f"Lowest SSIM: {metrics['SSIM'].min():0.3f} at row {metrics['SSIM'].idxmin()}")
    print(f"Lowest PSNR: {metrics['PSNR'].min():0.3f} at row {metrics['PSNR'].idxmin()}")
    print(f"Slowest Runtime: {metrics['runtime'].max():0.3f} at row {metrics['runtime'].idxmax()}")
    print("Average Metrics:")
    for column in metrics.columns:
        print(f"{column}: {metrics[column].mean():0.3f}")
    
    # print("---Max---")
    # for column in metrics.columns:
    #     print(f"{column}: {metrics[column].max():0.3f} at row {metrics[column].idxmax()}")
    # print("---Min---")
    # for column in metrics.columns:
    #     print(f"{column}: {metrics[column].min():0.3f} at row {metrics[column].idxmin()}")
    # print("---Mean---")

def main(verbose: bool = True, progress_bar: bool = True):
    parser = argparse.ArgumentParser(description='AUV Image Enhancement Comparison Tool')
    parser.add_argument('--input', '-i', type=str, default='input', 
                       help='Input directory containing images (default: input)')
    parser.add_argument('--output', '-o', type=str, default='output', 
                       help='Output directory for comparison images (default: output)')
    parser.add_argument('--image', type=str, help='Specific image file to process')
    parser.add_argument('--max-width', type=int, default=1200, 
                       help='Maximum width for output images (default: 1200)')
    
    parser.add_argument('--metrics-input', type=str, default='output',
                        help='Parse and display metrics in input directory (default: output)')
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
    
    if args.metrics_input:
        metrics_dir = Path(args.metrics_input)
        metric_files = list(metrics_dir.glob('*_metrics.txt'))
        for metric_file in metric_files:
            tabulate_metrics(Path(metric_file))
        return

    if not image_files:
        print(f"No images found in {input_dir}")
    
        return
    
    # Initialize enhancer
    enhancer = ie.ImageEnhancer(
        ie.UnderwaterColorCorrection(),
        ie.DCPEnhancement(),
        ie.CLAHEEnhancement(),
        ie.BilateralFilter()
    )

    # Define enhancement methods
    enhancement_methods = [
        ('CLAHE', ie.CLAHEEnhancement()),
        # ('Gamma Correction (γ=1.5)', ie.GammaCorrection(1.5)),
        ('White Balance (Gray World)', ie.WhiteBalance('gray_world')),
        # ('Red Enhancement', ie.RedChannelEnhancement(1.2)),
        ('Dark Channel Prior', ie.DCPEnhancement()),
        # ('Histogram Equalization', ie.HistogramEqualization()),
        # ('Unsharp Masking', ie.UnsharpMasking()),
        ('Underwater Color Correction', ie.UnderwaterColorCorrection()),
        ('Bilateral Filter', ie.BilateralFilter()),
        ('Combined Enhancement', enhancer)
    ]
    # initialize metric files
    with open(os.path.join(output_dir, "image_list.txt"), 'w') as f:
        f.write("")
    for i in range(len(enhancement_methods)):
        title = enhancement_methods[i][0]
        metrics_output_path = os.path.join(output_dir,f"{title}_metrics.txt")
        with open(metrics_output_path, 'w') as f:
            f.write("SSIM,PNSR,runtime\n")
        error_output_path = os.path.join(output_dir,f"{title}_errors.txt")
        with open(error_output_path, 'w') as f:
            f.write("")
    # Process each image
    for image_file in tqdm(image_files, disable=not progress_bar, leave=False,desc="Images"):
        print(f"Processing {image_file.name}...") if verbose else None
        image_output_dir = os.path.join(output_dir,image_file.stem)
        with open(os.path.join(output_dir, "image_list.txt"), 'a') as f:
            f.write(f"{image_file.name}\n")
        os.makedirs(image_output_dir, exist_ok=True)

        # Load image
        image = cv2.imread(str(image_file))
        if image is None:
            print(f"Error loading {image_file}")
            continue
        
        # Apply enhancements
        enhanced_images = []
        titles = []
        warnings.filterwarnings("error") # Treat warnings as errors
        for title, method in tqdm(enhancement_methods, disable=not progress_bar,leave=False, desc="Enhancements"):
            try:
                start_time = time.perf_counter()
                enhanced = method(image)
                runtime = time.perf_counter() - start_time
                enhanced_images.append(enhanced)
                titles.append(title)
                metrics_output_path = os.path.join(output_dir,f"{title}_metrics.txt")
                # fill out metrics
                with open(metrics_output_path, 'a') as f:
                    f.write(f"{test_ssim(image, enhanced)},{test_psnr(image, enhanced)},{runtime}\n")
                # Save enhanced image
                image_output_path = os.path.join(image_output_dir,f"{image_file.stem}_{title.replace(' ', '_')}.png")
                cv2.imwrite(image_output_path, enhanced)
            except Exception as e:
                error_output_path = os.path.join(output_dir,f"{title}_errors.txt")
                with open(error_output_path, 'w') as f:
                    f.write(f"Error or warning applying enhancement to {image_file.name}: {e}")
        warnings.filterwarnings("default") # restore warnings
        # Create comparison grid
        output_path = os.path.join(image_output_dir,f"comparison.png")
        create_comparison_grid(image, enhanced_images, titles, str(output_path), args.max_width)
        
        print(f"Comparison saved to {output_path}") if verbose else None
    
    print("Processing complete!") if verbose else None

if __name__ == "__main__":
    main(verbose=False, progress_bar=True)
    