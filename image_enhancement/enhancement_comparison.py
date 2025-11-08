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
from typing import List, Tuple, Callable
import os

class ImageEnhancer:
    """Class containing various image enhancement algorithms for underwater images."""
    
    @staticmethod
    def clahe_enhancement(image: np.ndarray, clip_limit: float = 2.0, tile_grid_size: Tuple[int, int] = (8, 8)) -> np.ndarray:
        """Apply Contrast Limited Adaptive Histogram Equalization (CLAHE)."""
        # Convert to LAB color space
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        # Apply CLAHE to L channel
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        l = clahe.apply(l)
        
        # Merge channels and convert back to BGR
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    @staticmethod
    def gamma_correction(image: np.ndarray, gamma: float = 1.5) -> np.ndarray:
        """Apply gamma correction to adjust brightness and contrast."""
        # Build lookup table
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")
        
        # Apply gamma correction
        return cv2.LUT(image, table)
    
    @staticmethod
    def white_balance(image: np.ndarray, method: str = 'gray_world') -> np.ndarray:
        """Apply white balance correction."""
        if method == 'gray_world':
            # Gray world assumption
            b, g, r = cv2.split(image.astype(np.float32))
            b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)
            gray_mean = (b_mean + g_mean + r_mean) / 3
            
            # Scale channels
            b = np.clip(b * (gray_mean / b_mean), 0, 255)
            g = np.clip(g * (gray_mean / g_mean), 0, 255)
            r = np.clip(r * (gray_mean / r_mean), 0, 255)
            
            return cv2.merge([b, g, r]).astype(np.uint8)
        
        elif method == 'white_patch':
            # White patch assumption - normalize to brightest pixel
            max_val = np.max(image)
            return np.clip(image * (255.0 / max_val), 0, 255).astype(np.uint8)
        
        return image
    
    @staticmethod
    def red_channel_enhancement(image: np.ndarray, enhancement_factor: float = 1.2) -> np.ndarray:
        """Enhance red channel to counteract underwater blue/green dominance."""
        b, g, r = cv2.split(image.astype(np.float32))
        
        # Enhance red channel
        r = np.clip(r * enhancement_factor, 0, 255)
        
        return cv2.merge([b, g, r]).astype(np.uint8)
    
    @staticmethod
    def dark_channel_prior(image: np.ndarray, window_size: int = 15, omega: float = 0.95, t0: float = 0.1) -> np.ndarray:
        """Apply Dark Channel Prior (DCP) dehazing algorithm."""
        # Convert to float
        img = image.astype(np.float32) / 255.0
        
        # Calculate dark channel
        def get_dark_channel(img, window_size):
            b, g, r = cv2.split(img)
            min_channel = np.minimum(np.minimum(b, g), r)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (window_size, window_size))
            dark_channel = cv2.erode(min_channel, kernel)
            return dark_channel
        
        dark_channel = get_dark_channel(img, window_size)
        
        # Estimate atmospheric light
        flat_dark = dark_channel.flatten()
        indices = np.argpartition(flat_dark, -int(0.001 * flat_dark.size))[-int(0.001 * flat_dark.size):]
        atmospheric_light = np.mean(img.reshape(-1, 3)[indices], axis=0)
        
        # Estimate transmission
        transmission = 1 - omega * get_dark_channel(img / atmospheric_light, window_size)
        transmission = np.maximum(transmission, t0)
        
        # Recover scene radiance
        result = np.zeros_like(img)
        for i in range(3):
            result[:, :, i] = (img[:, :, i] - atmospheric_light[i]) / transmission + atmospheric_light[i]
        
        return np.clip(result * 255, 0, 255).astype(np.uint8)
    
    @staticmethod
    def histogram_equalization(image: np.ndarray) -> np.ndarray:
        """Apply histogram equalization."""
        # Convert to YUV color space
        yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        y, u, v = cv2.split(yuv)
        
        # Apply histogram equalization to Y channel
        y = cv2.equalizeHist(y)
        
        # Merge channels and convert back to BGR
        yuv = cv2.merge([y, u, v])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    
    @staticmethod
    def unsharp_masking(image: np.ndarray, kernel_size: Tuple[int, int] = (5, 5), sigma: float = 1.0, amount: float = 1.0, threshold: int = 0) -> np.ndarray:
        """Apply unsharp masking for image sharpening."""
        # Create Gaussian blur
        blurred = cv2.GaussianBlur(image, kernel_size, sigma)
        
        # Calculate sharpened image
        sharpened = cv2.addWeighted(image, 1.0 + amount, blurred, -amount, 0)
        
        # Apply threshold
        if threshold > 0:
            low_contrast_mask = np.absolute(image - blurred) < threshold
            sharpened[low_contrast_mask] = image[low_contrast_mask]
        
        return sharpened
    
    @staticmethod
    def underwater_color_correction(image: np.ndarray) -> np.ndarray:
        """Apply underwater-specific color correction."""
        # Convert to float
        img = image.astype(np.float32) / 255.0
        
        # Underwater color correction matrix (approximate)
        correction_matrix = np.array([
            [1.2, -0.1, -0.1],  # Red channel
            [-0.1, 1.1, 0.0],   # Green channel
            [-0.2, -0.1, 1.3]   # Blue channel
        ])
        
        # Apply correction
        corrected = np.dot(img.reshape(-1, 3), correction_matrix.T).reshape(img.shape)
        
        return np.clip(corrected * 255, 0, 255).astype(np.uint8)
    
    @staticmethod
    def bilateral_filter_enhancement(image: np.ndarray, d: int = 9, sigma_color: float = 75, sigma_space: float = 75) -> np.ndarray:
        """Apply bilateral filtering for noise reduction while preserving edges."""
        return cv2.bilateralFilter(image, d, sigma_color, sigma_space)

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
    enhancer = ImageEnhancer()
    
    # Define enhancement methods
    enhancement_methods = [
        ('CLAHE', lambda img: enhancer.clahe_enhancement(img)),
        ('Gamma Correction (γ=1.5)', lambda img: enhancer.gamma_correction(img, 1.5)),
        ('White Balance (Gray World)', lambda img: enhancer.white_balance(img, 'gray_world')),
        ('Red Enhancement', lambda img: enhancer.red_channel_enhancement(img, 1.2)),
        ('Dark Channel Prior', lambda img: enhancer.dark_channel_prior(img)),
        ('Histogram Equalization', lambda img: enhancer.histogram_equalization(img)),
        ('Unsharp Masking', lambda img: enhancer.unsharp_masking(img)),
        ('Underwater Color Correction', lambda img: enhancer.underwater_color_correction(img)),
        ('Bilateral Filter', lambda img: enhancer.bilateral_filter_enhancement(img))
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
