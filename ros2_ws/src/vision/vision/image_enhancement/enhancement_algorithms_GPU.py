import torch
import kornia
import numpy as np
import os
from abc import ABC, abstractmethod
from typing import Optional
from enhancement_algorithms import ImageEnhancer, EnhancementAlgorithm

import DSC_pipeline as dsc_pipeline

class EnhancementAlgorithmGPU(EnhancementAlgorithm):
    def __init__(self, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
    
    def __str__(self):
        return f"Enhancement Algorithm on GPU: {self.algorithm_name()}"

    @abstractmethod
    def apply_algorithm(self, image: torch.Tensor, depth: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        Input: (B, C, H, W) float32 Tensor, Range [0, 1]
        Output: (B, C, H, W) float32 Tensor, Range [0, 1]
        Extra dimension for batch (since we process single images online, batch size = 1)
        """
        pass

    @torch.inference_mode()
    def __call__(self, image: torch.Tensor, depth: Optional[torch.Tensor] = None) -> torch.Tensor:
        return self.apply_algorithm(image, depth)

class GPUImageEnhancer(ImageEnhancer):
    """Aggregation of enhancement algorithms on GPU
    Handles conversion between numpy arrays and torch tensors on GPU.
    Thus enhancement algorithms can assume input and output are GPU torch tensors
    """
    def __init__(self, *algorithms, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.algorithms = list(algorithms)
        super().__init__("GPU", *algorithms)

    @torch.inference_mode()
    def enhance(self, image_np: np.ndarray, depth_np: Optional[np.ndarray] = None) -> np.ndarray:
        # np array (H,W,C) uint8 -> torch Tensor (1,C,H,W) float32, normalized to [0,1]
        tensor = (torch.from_numpy(image_np)
                  .to(self.device, dtype=torch.float32, non_blocking=True)
                  .permute(2, 0, 1)   # (H,W,C) -> (C,H,W) pytorch format
                  .unsqueeze_(0)      # (C,H,W) -> (1,C,H,W) for batch
                  .div_(255.0))

        # Convert depth if provided
        depth_tensor = None
        if depth_np is not None:
            depth_tensor = torch.from_numpy(depth_np).to(self.device, dtype=torch.float32, non_blocking=True)
            if depth_tensor.ndim == 2:
                depth_tensor = depth_tensor.unsqueeze(0).unsqueeze(0)  # (1, 1, H, W)
            elif depth_tensor.ndim == 3:
                depth_tensor = depth_tensor.unsqueeze(0)  # (1, C, H, W)

        for algo in self.algorithms:
            tensor = algo(tensor, depth_tensor)

        # (1,C,H,W) -> (H,W,C) uint8 numpy array
        return (tensor.squeeze(0)
                .permute(1, 2, 0)
                .mul_(255.0)
                .clamp_(0, 255)
                .to(dtype=torch.uint8)
                .cpu()
                .numpy())

# 1. COLOR CORRECTION ALGORITHMS

# 2. Dehazing REDUCTION ALGORITHMS

class DeepSeeColor(EnhancementAlgorithmGPU):
    """
    Deep Learning based Dehazing using the DeepSeeColor model and algorithm.
    Original Paper: 
    S. Jamieson, J. P. How and Y. Girdhar, "DeepSeeColor: Realtime Adaptive Color Correction for Autonomous Underwater Vehicles via Deep Learning Methods," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 3095-3101, doi: 10.1109/ICRA48891.2023.10160477.
    """
    def __init__(self, pipeline_model_path, device=None):
        super().__init__(device)
        
        if not os.path.exists(pipeline_model_path):
            raise FileNotFoundError(f"Pipeline Model Checkpoint not found at: {pipeline_model_path}")

        # Initialize Models
        self.pipeline = dsc_pipeline.DeepSeeColorPipeline().from_JIT_trace(pipeline_model_path, device=self.device)
        self.pipeline.eval()

    def apply_algorithm(self, image: torch.Tensor, depth: Optional[torch.Tensor] = None) -> torch.Tensor:
        with torch.no_grad():
            enhanced_rgb = self.pipeline(image, depth)
        return enhanced_rgb
    def algorithm_name(self) -> str:
        return "Dehazing - DeepSeeColor"
    
# 3. EDGE PRESERVATION ALGORITHMS

# 4. CONTRAST ENHANCEMENT ALGORITHMS

class CLAHEEnhancementGPU(EnhancementAlgorithmGPU):
    def __init__(self, clip_limit=2.0, grid_size=(8, 8), device=None):
        super().__init__(device)
        self.clip_limit = clip_limit
        self.grid_size = grid_size
        
    def apply_algorithm(self, image: torch.Tensor, depth: Optional[torch.Tensor] = None) -> torch.Tensor:
        return kornia.enhance.equalize_clahe(image, self.clip_limit, self.grid_size)

    def algorithm_name(self) -> str:
        return "Contrast Enhancement - CLAHE"