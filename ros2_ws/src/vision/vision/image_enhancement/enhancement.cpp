#include "enhancement.hpp"

#include <torch/script.h>
#include <torch/torch.h>

std::string CPUEnhancementAlgorithm::to_string() const
{
    return "Enhancement Algorithm on CPU: " + algorithm_name();
}

std::string  GPUEnhancementAlgorithm::to_string() const
{
    return "Enhancement Algorithm on GPU: " + algorithm_name();
}

CPUImageEnhancer::CPUImageEnhancer(const std::vector<const CPUEnhancementAlgorithm>& algorithms): algorithms{algorithms}
{

}

std::string CPUImageEnhancer::to_string() const
{
    return "Image Enhancer on CPU with algorithms:\n\t" + algorithms_to_string<CPUEnhancementAlgorithm>(algorithms);
}

cv::Mat CPUImageEnhancer::enhance(const cv::Mat& image) const
{
    cv::Mat enhanced_image = image.clone();
    for (const EnhancementAlgorithm& algorithm : algorithms)
    {
        enhanced_image = algorithm.apply_algorithm(enhanced_image);
    }
    return enhanced_image;
}

GPUImageEnhancer::GPUImageEnhancer(const std::vector<const GPUEnhancementAlgorithm>& algorithms): algorithms{algorithms}
{

};

std::string GPUImageEnhancer::to_string() const
{
    return "Image Enhancer on GPU with algorithms:\n\t" + algorithms_to_string<GPUEnhancementAlgorithm>(algorithms);
}



template<typename T>
std::string algorithms_to_string(const std::vector<const T>& algorithms)
{
    static_assert(std::is_base_of<EnhancementAlgorithm, T>::value, "Template parameter T must be derived from EnhancementAlgorithm");
    std::string algos;
    for (size_t i = 0; i < algorithms.size(); ++i)
    {
        algos += std::to_string(i + 1) + ". " + algorithms[i].to_string() + "\n\t";
    }
    return algos;
}