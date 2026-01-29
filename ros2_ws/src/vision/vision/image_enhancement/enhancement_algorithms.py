import cv2
import numpy as np
from typing import Tuple
from abc import ABC, abstractmethod

class EnhancementAlgorithm(ABC):
    """Abstract Base Class containing various image enhancement algorithms for underwater images."""
    @abstractmethod
    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def algorithm_name(self) -> str:
        pass

    def __call__(self, image: np.ndarray) -> np.ndarray:
        return self.apply_algorithm(image)

    def __str__(self):
        return f"Enhancement Algorithm: {self.algorithm_name()}"

class ImageEnhancer(ABC):
    def __init__(self,device_type, *algorithms):
        if not algorithms:
            self.algorithms = []
        else:
            self.algorithms = list(algorithms)
        self.type = device_type
    
    @abstractmethod
    def enhance(self, image: np.ndarray) -> np.ndarray:
        pass
    
    def __str__(self) -> str:
        algos = '\n\t'.join([f"{i}. {algo.algorithm_name()}" for i, algo in enumerate(self.algorithms,1)])
        return f"{self.type} Image Enhancer with algorithms [\n\t{algos}\n]"
    
    def __call__(self, image: np.ndarray) -> np.ndarray:
        return self.enhance(image)

class CPUImageEnhancer(ImageEnhancer):
    """Aggregation of enhancement algorithms on CPU
    Applies no enhancement by default.
    White balance and Guided Filter are commented out due to dependency issues
    with OpenCV.
    """
    def __init__(self, *algorithms):
        super().__init__("CPU", *algorithms)

    def enhance(self, image: np.ndarray) -> np.ndarray:
        for algorithm in self.algorithms:
            image = algorithm(image)
        return image

class Identity(EnhancementAlgorithm):
    """Applies no transformation"""

    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        return image

    def algorithm_name(self) -> str:
        return "Identity - No transformation"

# 1. COLOR CORRECTION ALGORITHMS

# class WhiteBalance(EnhancementAlgorithm):
#     """Apply white balance correction using gray world assumption."""

#     def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
#         wb = cv2.xphoto.createGrayworldWB()
#         return wb.balanceWhite(image)

#     def algorithm_name(self) -> str:
#         return "Color Correction - Gray World White Balance"


class RedChannelEnhancement(EnhancementAlgorithm):
    """Enhance red channel to counteract underwater blue/green dominance."""
    def __init__(self, enhancement_factor: float = 1.2):
        self.factor = enhancement_factor

    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        b, g, r = cv2.split(image.astype(np.float32))
        # Enhance red channel
        r = np.clip(r * self.factor, 0, 255)
        return cv2.merge([b, g, r]).astype(np.uint8)

    def algorithm_name(self) -> str:
        return "Color Correction - Red Channel Enhancement"


class UnderwaterColorCorrection(EnhancementAlgorithm):
    """Apply underwater-specific color correction."""
    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        # Underwater color correction matrix (approximate)
        correction_matrix = np.array([
            [1.2, -0.1, -0.1],  # Red channel
            [-0.1, 1.1, 0.0],   # Green channel
            [-0.2, -0.1, 1.3]   # Blue channel
        ], dtype=np.float32)

        # Apply correction using OpenCV
        return cv2.transform(image, correction_matrix)

    def algorithm_name(self) -> str:
        return "Color Correction - Underwater Color Correction"


# 2. BACKSCATTER REDUCTION ALGORITHMS

class DCPEnhancement(EnhancementAlgorithm):
    """Apply Dark Channel Prior (DCP) dehazing algorithm."""
    def __init__(self, window_size: int = 15, omega: float = 0.95, t0: float = 0.1):
        self.window_size = window_size
        self.omega = omega
        self.t0 = t0

    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        # Convert to float
        img = image.astype(np.float32) / 255.0
        dark_channel = self.get_dark_channel(img)

        # Estimate atmospheric light
        flat_dark = dark_channel.flatten()
        num_pixels = max(1, int(0.001 * flat_dark.size))
        indices = np.argpartition(flat_dark, -num_pixels)[-num_pixels:]
        atmospheric_light = np.mean(img.reshape(-1, 3)[indices], axis=0)

        # Estimate transmission
        transmission = 1 - self.omega * self.get_dark_channel(img / atmospheric_light)
        transmission = np.maximum(transmission, self.t0)

        # Recover scene radiance
        result = np.zeros_like(img)
        for i in range(3):
            result[:, :, i] = (img[:, :, i] - atmospheric_light[i]) / transmission + atmospheric_light[i]

        return np.clip(result * 255, 0, 255).astype(np.uint8)

    # Calculate dark channel
    def get_dark_channel(self, img):
        b, g, r = cv2.split(img)
        min_channel = np.minimum(np.minimum(b, g), r)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.window_size, self.window_size))
        dark_channel = cv2.erode(min_channel, kernel)
        return dark_channel

    def algorithm_name(self) -> str:
        return "Dehazing - Dark Channel Prior"


# 3. EDGE PRESERVATION ALGORITHMS

# class GuidedFilter(EnhancementAlgorithm):
#     """Apply guided filter for edge-preserving smoothing using OpenCV implementation."""

#     def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
#         # Default parameters
#         radius = 8
#         epsilon = 0.01

#         # Convert to grayscale guide
#         if image.ndim == 3:
#             guide = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         else:
#             guide = image

#         # Apply guided filter using OpenCV
#         result = cv2.ximgproc.guidedFilter(guide, image, radius, epsilon)

#         return result

#     def algorithm_name(self) -> str:
#         return "Edge Preservation - Guided Filter"


# 4. CONTRAST ENHANCEMENT ALGORITHMS

class CLAHEEnhancement(EnhancementAlgorithm):
    """Apply Contrast Limited Adaptive Histogram Equalization (CLAHE)."""
    def __init__(self, clip_limit: float = 2.0, tile_grid_size: Tuple[int, int] = (8, 8)):
        self._clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)

    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        # Convert to LAB color space
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        # Apply CLAHE to L channel
        l = self._clahe.apply(l)

        # Merge channels and convert back to BGR
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def algorithm_name(self) -> str:
        return "Contrast Enhancement - CLAHE"


class GammaCorrection(EnhancementAlgorithm):
    """Apply gamma correction to adjust brightness and contrast."""
    def __init__(self, gamma: float = 1.5):
        inv_gamma = 1.0 / gamma
        self.table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")

    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        # Apply gamma correction
        return cv2.LUT(image, self.table)

    def algorithm_name(self) -> str:
        return "Contrast Enhancement - Gamma Correction"


class HistogramEqualization(EnhancementAlgorithm):
    """Apply histogram equalization."""
    def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
        # Apply histogram equalization directly using OpenCV
        return cv2.equalizeHist(image)

    def algorithm_name(self) -> str:
        return "Contrast Enhancement - Histogram Equalization"
