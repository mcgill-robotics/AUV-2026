import argparse

import torch
import torch.nn as nn
import torch.nn.functional as F
import os
import numpy as np
import cv2

import DeepSeeColor as dsc

class DeepSeeColorPipeline(nn.Module):
    """
    Wrapper for entire DeepSeeColor pipeline (BS + DA) that can be used as a single enhancement algorithm in GPUImageEnhancer.
    Only BackscatterNet and DeattenuateNet need to be trained, the pipeline itself is fixed and deterministic. This allows us to JIT compile the entire pipeline for maximum efficiency on GPU.
    Note: The original DeepSeeColor paper used a custom morphological closing operation on the depth map before feeding it into the BS and DA models. We will implement this closing operation as a differentiable function within the pipeline to maintain the end-to-end GPU processing.
    It takes raw image and depth tensors, and returns the final enhanced tensor.
    """
    epsilon: torch.Tensor  # Type hint for registered buffer
    
    def __init__(self, closing_kernel=7):
        super().__init__()
        # Pre-calculate morphology parameters
        self.kernel_size = closing_kernel
        self.pad = closing_kernel // 2
        
        # since we will be doing a lot of clamping to 1/255, we can pre-store that as a buffer to avoid overhead
        self.register_buffer('epsilon', torch.tensor(1.0 / 255.0))

    @classmethod
    def from_JIT_trace(cls, model_path, device=None):
        """
        Load a JIT-traced pipeline model from the specified path.
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"JIT Pipeline Model not found at: {model_path}")
        if device is None:
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')     
        return torch.jit.load(model_path).to(device)
    
    def load_backscatter_weights(self, bs_ckpt_path):
        if not os.path.exists(bs_ckpt_path):
            raise FileNotFoundError(f"BackscatterNet Model Checkpoint not found at: {bs_ckpt_path}")
        # backscatter net as submodule of pipeline
        self.bs_model = dsc.BackscatterNet()
        self.bs_model.load_state_dict(torch.load(bs_ckpt_path))
        self.bs_model.eval()
        
    def load_deattenuate_weights(self, da_ckpt_path):
        if not os.path.exists(da_ckpt_path):
            raise FileNotFoundError(f"DeattenuateNet Model Checkpoint not found at: {da_ckpt_path}")
        # deattenuate net as submodule of pipeline
        self.da_model = dsc.DeattenuateNet()
        self.da_model.load_state_dict(torch.load(da_ckpt_path))
        self.da_model.eval()

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
    
    pipeline = DeepSeeColorPipeline().to(device)
    
    pipeline.load_backscatter_weights(bs_model_path)
    pipeline.load_deattenuate_weights(da_model_path)
    pipeline.to(device)
    pipeline.eval()
    
    dummy_img = torch.rand(1, 3, height, width).to(device)
    dummy_depth = torch.rand(1, 1, height, width).to(device)
    
    return torch.jit.trace(pipeline, (dummy_img, dummy_depth))

def test_pipeline(pipeline_model_path, test_image_path, test_depth_path,output_path):
    device = torch.device('cuda')
    
    # Load and preprocess test image and depth
    img = cv2.imread(test_image_path)  # Assuming 3-channel RGB image
    depth = cv2.imread(test_depth_path, cv2.IMREAD_ANYDEPTH)  # Assuming single-channel depth image
    
    img_tensor = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0).to(device) / 255.0
    depth_tensor = torch.from_numpy(depth).float().unsqueeze(0).unsqueeze(0).to(device) / 1000.0
    
    pipeline = DeepSeeColorPipeline.from_JIT_trace(pipeline_model_path, device=device)
    pipeline.eval()
    with torch.no_grad():
        output = pipeline(img_tensor[:, :3, :, :], depth_tensor)
    
    output_img = (output.squeeze(0).permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
    cv2.imwrite(output_path, output_img)
    print(f"Test output saved as {output_path}")

def main():
    parser = argparse.ArgumentParser(
        description="Export BackscatterNet and DeattenuateNet as a single JIT-traceable pipeline for GPU inference.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 DSC_pipeline.py --bs-model-path path/to/bs_model.pt --da-model-path path/to/da_model.pt --output-path dsc_pipeline.pt
    python3 DSC_pipeline.py --bs-model-path path/to/bs_model.pt --da-model-path path/to/da_model.pt --output-path dsc_pipeline.pt --sim --test-image-path path/to/test_image.png --test-depth-path path/to/test_depth.png
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
    
    parser.add_argument(
        "--test-image-path",
        type=str,
        default=None,
        help="Optional path to a test RGB image (4-channel .jpg) to run through the pipeline after export for verification. Must be used in conjunction with --test-depth-path."
    )
    
    parser.add_argument(
        "--test-depth-path",
        type=str,
        default=None,
        help="Optional path to a test depth image (single-channel .png) to run through the pipeline after export for verification. Must be used in conjunction with --test-image-path."
    )
    
    parser.add_argument(
        "--test-output-path",
        type=str,
        default="dsc_test_output.png",
        help="Path to save the test output image after running the pipeline on the test image and depth. Default is dsc_test_output.png."
    )
    args = parser.parse_args()
    
    height = 600 if args.sim else 376
    width = 960 if args.sim else 672
    
    pipeline = export_pipeline(args.bs_model_path, args.da_model_path, height, width)
    torch.jit.save(pipeline, args.output_path)
    print(f"Pipeline exported and saved to {args.output_path}")
    
    if args.test_image_path and args.test_depth_path:
        print("Running test on provided image and depth...")
        test_pipeline(args.output_path, args.test_image_path, args.test_depth_path,args.test_output_path)
    
if __name__ == "__main__":
    main()