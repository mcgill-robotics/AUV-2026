
import cv2
import numpy as np
from typing import Tuple
from abc import ABC, abstractmethod
# optional PyTorch support
try:
        import torch
        import torchvision.transforms as T
        TORCH_INSTALLED = True
except ImportError:
        TORCH_INSTALLED = False

if TORCH_INSTALLED:
        class TorchRBGTensorToCVBGRNumpy:
                """Convert torch RGB tensor [C,H,W] in [0,1] to np.ndarray in BGR uint8."""
                def __call__(self, tensor: torch.Tensor) -> np.ndarray:
                        array = tensor.permute(1, 2, 0).detach().cpu().numpy()
                        array = (array * 255).clip(0, 255).astype(np.uint8)
                        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

        class CVBGRNumpyToTorchRGBTensor:
                """Convert np.ndarray BGR uint8 -> torch RGB float tensor [C,H,W]."""
                def __call__(self, array: np.ndarray) -> torch.Tensor:
                        rgb = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
                        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
                        return tensor


class EnhancementAlgorithm(ABC):
        '''Abstract Base Class containing various image enhancement algorithms for underwater images.'''
        @abstractmethod
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                pass

        def __call__(self, image: np.ndarray) -> np.ndarray:
                return self.apply_algorithm(image)
        
class ImageEnhancer:
        '''Aggregation of enhancement algorithms'''
        def __init__ (self,*algorithms: EnhancementAlgorithm):
                self.algorithms = list(algorithms)

        def enhance(self, image: np.ndarray) -> np.ndarray:
                for algorithm in self.algorithms:
                        image = algorithm(image)
                return image
        def to_torch_transform(self):
                '''Return a torch transform compose equivalent (calls enhancement on CPU).'''
                if not TORCH_INSTALLED:
                        raise NotImplementedError("PyTorch is not installed.")
                else:
                        return T.Compose([
                        TorchRBGTensorToCVBGRNumpy(),
                        self, # Call the enhance method
                        CVBGRNumpyToTorchRGBTensor(),
                        ])
        def __call__(self, image: np.ndarray) -> np.ndarray:
                return self.enhance(image)
        
class CLAHEEnhancement(EnhancementAlgorithm):
        """Apply Contrast Limited Adaptive Histogram Equalization (CLAHE)."""
        def __init__(self,clip_limit: float = 2.0, tile_grid_size: Tuple[int, int] = (8, 8)):
                self._clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
                                
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                # Convert to LAB color space
                lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                
                # Apply CLAHE to L channel
                l = self._clahe.apply(l)
                
                # Merge channels and convert back to BGR
                lab = cv2.merge([l, a, b])
                return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

class GammaCorrection(EnhancementAlgorithm):
        """Apply gamma correction to adjust brightness and contrast."""
        def __init__(self,gamma: float = 1.5):
                inv_gamma = 1.0 / gamma
                self.table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")
                
        def apply_algorithm(self, image: np.array) -> np.ndarray:             
                # Apply gamma correction
                return cv2.LUT(image, self.table)
class WhiteBalance(EnhancementAlgorithm):
        """Apply white balance correction using gray world assumption."""
        
        def apply_algorithm(self, image: np.ndarray) -> np.ndarray:
            # Gray world assumption
            b, g, r = cv2.split(image.astype(np.float32))
            b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)
            gray_mean = (b_mean + g_mean + r_mean) / 3
            
            # Scale channels
            b = np.clip(b * (gray_mean / b_mean), 0, 255)
            g = np.clip(g * (gray_mean / g_mean), 0, 255)
            r = np.clip(r * (gray_mean / r_mean), 0, 255)
            
            return cv2.merge([b, g, r]).astype(np.uint8)
                
class RedChannelEnhancement(EnhancementAlgorithm):
        """Enhance red channel to counteract underwater blue/green dominance."""
        def __init__(self,enhancement_factor: float = 1.2):
                self.factor = enhancement_factor
                
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                b, g, r = cv2.split(image.astype(np.float32))
                # Enhance red channel
                r = np.clip(r * self.factor, 0, 255)
                return cv2.merge([b, g, r]).astype(np.uint8)
                        
class DCPEnhancement(EnhancementAlgorithm):
        """Apply Dark Channel Prior (DCP) dehazing algorithm."""
        def __init__(self,window_size: int = 15, omega: float = 0.95, t0: float = 0.1):
                self.window_size = window_size
                self.omega = omega
                self.t0 = t0
        
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                # Convert to float
                img = image.astype(np.float32) / 255.0
                dark_channel = self.get_dark_channel(img)
                
                # Estimate atmospheric light
                flat_dark = dark_channel.flatten()
                indices = np.argpartition(flat_dark, -int(0.001 * flat_dark.size))[-int(0.001 * flat_dark.size):]
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
        def get_dark_channel(self,img):
            b, g, r = cv2.split(img)
            min_channel = np.minimum(np.minimum(b, g), r)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.window_size, self.window_size))
            dark_channel = cv2.erode(min_channel, kernel)
            return dark_channel
    
class HistogramEqualization(EnhancementAlgorithm):
        """Apply histogram equalization."""
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                # Convert to YUV color space
                yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
                y, u, v = cv2.split(yuv)
                
                # Apply histogram equalization to Y channel
                y = cv2.equalizeHist(y)
                
                # Merge channels and convert back to BGR
                yuv = cv2.merge([y, u, v])
                return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
        

class UnsharpMasking(EnhancementAlgorithm):
        """Apply unsharp masking for image sharpening."""
        def __init__(self,kernel_size: Tuple[int, int] = (5, 5), sigma: float = 1.0, sharpening_amount: float = 1.0, threshold: int = 0):
               self.kernel_size = kernel_size
               self.sigma = sigma
               self.amount = sharpening_amount
               self.threshold = threshold
        
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                # Create Gaussian blur
                blurred = cv2.GaussianBlur(image, self.kernel_size, self.sigma)
                
                # Calculate sharpened image
                sharpened = cv2.addWeighted(image, 1.0 + self.amount, blurred, -self.amount, 0)
                
                # Apply threshold
                if self.threshold > 0:
                        low_contrast_mask = np.absolute(      image - blurred) < self.threshold
                        sharpened[low_contrast_mask] = image[low_contrast_mask]
                
                return sharpened

class UnderwaterColorCorrection(EnhancementAlgorithm):
        """Apply underwater-specific color correction."""
        def apply_algorithm(self, image: np.array) -> np.ndarray:
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
class BilateralFilter(EnhancementAlgorithm):
        """Apply bilateral filtering for noise reduction while preserving edges."""
        def __init__(self,d: int = 9, sigma_color: float = 75, sigma_space: float = 75):
                self.d = d
                self.sigma_color = sigma_color
                self.sigma_space = sigma_space
        def apply_algorithm(self, image: np.array) -> np.ndarray:
                return cv2.bilateralFilter(image, self.d, self.sigma_color, self.sigma_space)
