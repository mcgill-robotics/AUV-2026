import argparse

import torch
import torch.nn as nn
import torch.nn.functional as F

import DeepSeeColor as dsc

class DeepSeeColorPipeline(nn.Module):
    """
    Wrapper for entire DeepSeeColor pipeline (BS + DA) that can be used as a single enhancement algorithm in GPUImageEnhancer.
    Only BackscatterNet and DeattenuateNet need to be trained, the pipeline itself is fixed and deterministic. This allows us to JIT compile the entire pipeline for maximum efficiency on GPU.
    Note: The original DeepSeeColor paper used a custom morphological closing operation on the depth map before feeding it into the BS and DA models. We will implement this closing operation as a differentiable function within the pipeline to maintain the end-to-end GPU processing.
    It takes raw image and depth tensors, and returns the final enhanced tensor.
    """
    epsilon: torch.Tensor  # Type hint for registered buffer
    
    def __init__(self, bs_model_path, da_model_path, closing_kernel=7):
        super().__init__()
        
        # backscatter net as submodule of pipeline
        self.bs_model = dsc.BackscatterNet()
        self.bs_model.load_state_dict(torch.load(bs_model_path))
        self.bs_model.eval()
        
        # deattenuate net as submodule of pipeline
        self.da_model = dsc.DeattenuateNet()
        self.da_model.load_state_dict(torch.load(da_model_path))
        self.da_model.eval()
        
        # Pre-calculate morphology parameters
        self.kernel_size = closing_kernel
        self.pad = closing_kernel // 2
        
        # since we will be doing a lot of clamping to 1/255, we can pre-store that as a buffer to avoid overhead
        self.register_buffer('epsilon', torch.tensor(1.0 / 255.0))

    def _morphological_closing(self, depth):
        # Native PyTorch Closing (Dilation -> Erosion), this mimics the kornia morphology.closing but is fully differentiable and has no overhead from switching to CPU or using kornia. We use max pooling to achieve dilation and erosion.
        dilated = F.max_pool2d(depth, kernel_size=self.kernel_size, stride=1, padding=self.pad)
        eroded = -F.max_pool2d(-dilated, kernel_size=self.kernel_size, stride=1, padding=self.pad)
        return eroded

    def forward(self, img, depth):
        """
        img: (1, 3, H, W) float tensor [0.0, 1.0]
        depth: (1, 1, H, W) float tensor [meters]
        """
        # Preprocess Depth with morphological closing (GPU)
        clean_depth = self._morphological_closing(depth)

        # Backscatter Estimation (backscatter) only used for visualization
        direct, _ = self.bs_model(img, clean_depth)

        # The "Glue" Logic (Z-Score & Clamping) entirely in graph
        direct_mean = direct.mean(dim=[2, 3], keepdim=True)
        direct_std = direct.std(dim=[2, 3], unbiased=False, keepdim=True)

        direct_z = (direct - direct_mean) / (direct_std + 1e-8)
        clamped_z = torch.clamp(direct_z, min=-5.0, max=5.0)

        # Reconstruct input for DA model
        da_input = torch.clamp(
            (clamped_z * direct_std) + torch.maximum(direct_mean, self.epsilon), 
            min=0.0, max=1.0
        )

        # Similar to backscatter, f is only used for visualization
        _, final_img = self.da_model(da_input, clean_depth)

        # Final Clamp
        final_img = torch.clamp(final_img, min=0.0, max=1.0)
        
        return final_img

def export_pipeline(bs_model_path, da_model_path, height=600,width=960):
    print("Exporting DeepSeeColor pipeline with:")
    print(f"  - BackscatterNet model: {bs_model_path}")
    print(f"  - DeattenuateNet model: {da_model_path}")
    print(f"  - Image size: {width}x{height}")
    device = torch.device('cuda')
    
    pipeline = DeepSeeColorPipeline(bs_model_path, da_model_path).to(device)
    pipeline.eval()
    
    dummy_img = torch.rand(1, 3, height, width).to(device)
    dummy_depth = torch.rand(1, 1, height, width).to(device)
    
    return torch.jit.trace(pipeline, (dummy_img, dummy_depth))

def main():
    parser = argparse.ArgumentParser(
        description="Export BackscatterNet and DeattenuateNet as a single JIT-traceable pipeline for GPU inference.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 DSC_pipeline.py --bs-model-path path/to/bs_model.pt --da-model-path path/to/da_model.pt --output-path dsc_pipeline.pt
        """
        )
    
    parser.add_argument(
        "--bs-model-path",
        type=str,
        required=True,
        help="Path to the trained BackscatterNet model checkpoint (state_dict)."
    )
    parser.add_argument(
        "--da-model-path",
        type=str,
        required=True,
        help="Path to the trained DeattenuateNet model checkpoint (state_dict)."
    )
    parser.add_argument(
        "--output-path",
        type=str,
        required=True,
        help="Path to save the exported JIT pipeline (e.g., dsc_pipeline.pt)."
    )
    
    parser.add_argument(
        "--sim",
        action='store_true',
        help="Models take simulation image, i.e. 960x600 instead of 672x376."
    )
    
    args = parser.parse_args()
    
    height = 600 if args.sim else 376
    width = 960 if args.sim else 672
    
    pipeline = export_pipeline(args.bs_model_path, args.da_model_path, height, width)
    torch.jit.save(pipeline, args.output_path)
    print(f"Pipeline exported and saved to {args.output_path}")
    
if __name__ == "__main__":
    main()