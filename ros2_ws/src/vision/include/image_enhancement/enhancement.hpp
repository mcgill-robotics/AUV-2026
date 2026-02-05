#pragma once
#include <opencv2/opencv.hpp>


class EnhancementAlgorithm
{
public:
    EnhancementAlgorithm() = default;
    virtual ~EnhancementAlgorithm() = default;
    virtual std::string to_string() const = 0;
    virtual cv::Mat apply_algorithm(const cv::Mat& image) const = 0;
    
protected:
    virtual std::string algorithm_name() const = 0;

};

class CPUEnhancementAlgorithm : public EnhancementAlgorithm
{
public:
    CPUEnhancementAlgorithm() = default;
    ~CPUEnhancementAlgorithm() override = default;
    std::string to_string() const override;
protected:
    cv::Mat apply_algorithm(const cv::Mat& image) const override = 0;
    std::string algorithm_name() const override = 0;
};

class GPUEnhancementAlgorithm : public EnhancementAlgorithm
{
public:
    GPUEnhancementAlgorithm() = default;
    ~GPUEnhancementAlgorithm() override = default;
    std::string to_string() const override;
protected:
    cv::Mat apply_algorithm(const cv::Mat& image) const override = 0;
    std::string algorithm_name() const override = 0;
};

class CPUImageEnhancer
{
public:
    CPUImageEnhancer(const std::vector<const CPUEnhancementAlgorithm>& algorithms = {});
    ~CPUImageEnhancer() = default;
    cv::Mat enhance(const cv::Mat& image) const;
    std::string to_string() const;
private:
    std::vector<const CPUEnhancementAlgorithm> algorithms;
};

class GPUImageEnhancer
{
public: 
    GPUImageEnhancer(const std::vector<const GPUEnhancementAlgorithm>& algorithms = {});
    ~GPUImageEnhancer() = default;
    cv::Mat enhance(const cv::Mat& image) const;
    std::string to_string() const;
private:
    std::vector<const GPUEnhancementAlgorithm> algorithms;
};

template<typename T>
std::string algorithms_to_string (const std::vector<const T>& algorithms);