import torch
import kornia
import numpy as np
from abc import ABC, abstractmethod
from enhancement_algorithms import ImageEnhancer

class EnhancementAlgorithmGPU(ABC):
    def __init__(self, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)

    @abstractmethod
    def apply_algorithm(self, image: torch.Tensor) -> torch.Tensor:
        """Extra dimension for batch (since we process single images online, batch size = 1)
        So input image shape: (1, C, H, W) as PyTorch tensor
        """
        pass

    def __call__(self, image: torch.Tensor) -> torch.Tensor:
        return self.apply_algorithm(image)

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

    def enhance(self, image_np: np.ndarray) -> np.ndarray:
        # np array (H,W,C) -> torch Tensor (C,H,W) and move to GPU
        tensor = kornia.utils.image_to_tensor(image_np).float() / 255.0
        # move to GPU and add batch dimension (1,C,H,W)
        tensor = tensor.to(self.device).unsqueeze(0)

        for algo in self.algorithms:
            tensor = algo(tensor)

        # tensor (1,C,H,W) on GPU -> np array (H,W,C) on CPU
        enhanced_np = kornia.utils.tensor_to_image(tensor.squeeze(0).cpu())
        return (enhanced_np * 255).astype(np.uint8)