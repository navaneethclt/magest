#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <onnxruntime_cxx_api.h>

#include<optional>
#include <codecvt>
#include <fstream>
#include <utility>
#include <thread>




class onnxnndetect {
private:
    Ort::Env env;
    Ort::Session session{ nullptr }; // Declare session here
    Ort::SessionOptions sessionOptions;
    Ort::AllocatorWithDefaultOptions allocator;
    std::optional<Ort::AllocatedStringPtr> input_name_;
    std::optional<Ort::AllocatedStringPtr> output_name_;
    std::vector<Ort::Value> inputTensors;

    std::wstring charToWstring(const char* str)
    {
        typedef std::codecvt_utf8<wchar_t> convert_type;
        std::wstring_convert<convert_type, wchar_t> converter;

        return converter.from_bytes(str);
    }

    // Prepare input tensor data
    std::array<float, 2> inputData = { 1.0f, 2.0f }; // Input data for a tensor with shape [1, 2]
    std::array<int64_t, 2> inputShape{ 1, 2 };


    // Get input tensor information
    size_t inputTensorSize = inputData.size() * sizeof(float);
public:

    onnxnndetect(const std::string& modelPath)
    {
        env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");

        sessionOptions = Ort::SessionOptions();

        std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
        auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
        OrtCUDAProviderOptions cudaOption;

        if (true && (cudaAvailable == availableProviders.end())) {
            std::cout << "GPU is not supported by your ONNXRuntime build. Fallback to CPU." << std::endl;
            std::cout << "Inference device: CPU" << std::endl;
        }
        else if (true && (cudaAvailable != availableProviders.end())) {
            std::cout << "Inference device: GPU" << std::endl;
            sessionOptions.AppendExecutionProvider_CUDA(cudaOption);
            sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            unsigned int numCores = std::thread::hardware_concurrency();
            std::cout << "Number of CPU cores: " << numCores << std::endl;
            // sessionOptions.SetIntraOpNumThreads(1);

        }
        else {
            std::cout << "Inference device: CPU" << std::endl;
        }


#ifdef _WIN32
        std::wstring w_modelPath = charToWstring(modelPath.c_str());
        session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
#else
        session = Ort::Session(env, modelPath.c_str(), sessionOptions);
#endif


        Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
        std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
        // checking if width and height are dynamic
        if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1) {
            std::cout << "Dynamic input shape" << std::endl;
        }

        for (auto shape : inputTensorShape)
            std::cout << "Input shape: " << shape << std::endl;

        input_name_.emplace(session.GetInputNameAllocated(0, allocator));
        output_name_.emplace(session.GetOutputNameAllocated(0, allocator));

    }

    std::vector<float>  detect(std::array<float, 2> inputData) {
        // Prepare input tensor from the provided input data
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);

        std::vector<Ort::Value> inputTensors;

        inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputData.data(), inputData.size(), inputShape.data(), inputShape.size()));

        // Run inference
        const char* inputNames[] = { input_name_->get() };
        const char* outputNames[] = { output_name_->get() };

        std::vector<Ort::Value> outputTensors = session.Run(Ort::RunOptions{}, inputNames, inputTensors.data(), std::size(inputNames), outputNames, std::size(inputNames));

        // Process output tensors if needed


        auto* rawOutput = outputTensors[0].GetTensorData<float>();
        std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
        size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        std::vector<float> output(rawOutput, rawOutput + count);


       /* auto* rawInput = inputTensors[0].GetTensorData<float>();
        std::vector<int64_t> inputShape = inputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
        count = inputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        std::vector<float> input(rawInput, rawInput + count);*/


        return output;
    }
};


