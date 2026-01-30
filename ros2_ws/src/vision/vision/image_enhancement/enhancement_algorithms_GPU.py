import torch
import kornia
import numpy as np
from abc import ABC, abstractmethod
from enhancement_algorithms import ImageEnhancer, EnhancementAlgorithm

class EnhancementAlgorithmGPU(EnhancementAlgorithm):
    def __init__(self, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
    
    def __str__(self):
        return f"Enhancement Algorithm on GPU: {self.algorithm_name()}"

    @abstractmethod
    def apply_algorithm(self, image: torch.Tensor) -> torch.Tensor:
        """
        Input: (B, C, H, W) float32 Tensor, Range [0, 1]
        Output: (B, C, H, W) float32 Tensor, Range [0, 1]
        Extra dimension for batch (since we process single images online, batch size = 1)
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
        # clip to 0,1 and convert back to uint8
        return (np.clip(enhanced_np, 0, 1) * 255).astype(np.uint8)

# 1. COLOR CORRECTION ALGORITHMS

# 2. Dehazing REDUCTION ALGORITHMS

class DeepSeeColor(EnhancementAlgorithmGPU):
    """
    Deep Learning based Dehazing using the DeepSeeColor model and algorithm.
    Original Paper: 
    S. Jamieson, J. P. How and Y. Girdhar, "DeepSeeColor: Realtime Adaptive Color Correction for Autonomous Underwater Vehicles via Deep Learning Methods," 2023 IEEE International Conference on Robotics and Automation (ICRA), London, United Kingdom, 2023, pp. 3095-3101, doi: 10.1109/ICRA48891.2023.10160477.
    """
    def __init__(self, bs_model_class, da_model_class, 
                 bs_ckpt_path: str, da_ckpt_path: str, device=None):
        super().__init__(device)
        
        if not os.path.exists(bs_ckpt_path):
            raise FileNotFoundError(f"BS Model Checkpoint not found at: {bs_ckpt_path}")
        if not os.path.exists(da_ckpt_path):
            raise FileNotFoundError(f"DA Model Checkpoint not found at: {da_ckpt_path}")

        # Initialize Models
        self.bs_model = bs_model_class().to(self.device)
        self.da_model = da_model_class().to(self.device)
        
        # Load Weights
        self.bs_model.load_state_dict(torch.load(bs_ckpt_path, map_location=self.device))
        self.da_model.load_state_dict(torch.load(da_ckpt_path, map_location=self.device))
        
        self.bs_model.eval()
        self.da_model.eval()

        # Pre-allocate kernel for depth morphology to avoid overhead
        self.morph_kernel = torch.ones(3, 3, device=self.device)

    def apply_algorithm(self, image: torch.Tensor) -> torch.Tensor:
        # Note: This pipeline requires DEPTH.
        # The abstract base class `apply_algorithm` only accepts `image`.
        # OPTION A: We change the architecture to pass kwargs.
        # OPTION B: We assume depth is stacked in the image tensor (4 channels: RGBD).
        # For this implementation, let's assume the input is RGBD (4 channels).
        
        if image.shape[1] == 4:
            rgb = image[:, :3, :, :]
            depth = image[:, 3:4, :, :] # Keep channel dim: (B, 1, H, W)
        else:
            # Fallback/Warning if no depth provided. 
            # This specific algorithm CANNOT run without depth.
            raise ValueError("DeepLearningDehazingGPU requires a 4-channel input (RGB+Depth).")

        # 1. Depth Pre-processing (GPU)
        # Assume depth is normalized [0,1] or needs scaling? 
        # Based on previous code: depth was / 1000.0. 
        # Here we assume the input tensor is already prepared or we scale it.
        # Let's assume input depth channel is raw meters for safety or check max value.
        
        # Morphological Closing on Depth
        depth = kornia.morphology.closing(depth, self.morph_kernel)

        # 2. BS Model
        direct, backscatter = self.bs_model(rgb, depth)

        # 3. Intermediate Processing (Z-Score & Clamping)
        direct_mean = direct.mean(dim=[2, 3], keepdim=True)
        direct_std = direct.std(dim=[2, 3], keepdim=True)
        
        # Add epsilon to std to prevent div by zero
        direct_z = (direct - direct_mean) / (direct_std + 1e-8)
        clamped_z = torch.clamp(direct_z, -5, 5)

        epsilon = torch.tensor([1. / 255], device=self.device)
        direct_input_da = torch.clamp(
            (clamped_z * direct_std) + torch.maximum(direct_mean, epsilon), 0, 1
        )

        # 4. DA Model
        f, J = self.da_model(direct_input_da, depth)

        # Return the corrected image J.
        # We clamp to ensure valid range.
        return torch.clamp(J, 0, 1)

    def algorithm_name(self) -> str:
        return "Dehazing - DeepSeeColor"
    
# 3. EDGE PRESERVATION ALGORITHMS

# 4. CONTRAST ENHANCEMENT ALGORITHMS

class CLAHEEnhancementGPU(EnhancementAlgorithmGPU):
    def __init__(self, clip_limit=2.0, grid_size=(8, 8), device='cuda'):
        self.clip_limit = clip_limit
        self.grid_size = grid_size
        super().__init__(device)
        
    def apply_algorithm(self, image: torch.Tensor) -> torch.Tensor:
        return kornia.enhance.equalize_clahe(image, self.clip_limit, self.grid_size)

    def algorithm_name(self) -> str:
        return "Contrast Enhancement - CLAHE"