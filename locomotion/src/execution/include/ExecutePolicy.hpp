#pragma once

#include <thread>

#include "QuadROSComm.hpp"
#include <torch/torch.h>
#include <torch/script.h>

class Policy {
public:
    Policy(const std::string& path_label) {
        body = torch::jit::load(path + "/checkpoints/body_latest.jit");
        adaptation_module = torch::jit::load(path + "/checkpoints/adaptation_module_latest.jit");
    }

    torch::Tensor forward(const torch::Tensor& obs_history) {
        torch::Tensor latent = adaptation_module.forward({obs_history.to(at::kCPU)});
        torch::Tensor action = body.forward({torch::cat({obs_history.to(at::kCPU), latent}, -1)});
        info["latent"] = latent;
        return action;
    }

private:
    torch::jit::script::Module body;
    torch::jit::script::Module adaptation_module;
}

class ExecutorPolicy {
public:
    Executor();
    Executor(const std::string& name);
    ~Executor();

    void setUpdateRate(const float& rate);

    void load_policy(const std::string& path_label);

    void step();

    void run();

private:
    std::string m_name;
    float m_rate;
    float m_dt;
    int delay_ms = 10;

    void InitClass();

    // SHM m_plant_data;
    std::shared_ptr<QuadROSComm> m_plant_data_ptr;

    QuadrupedSensorData m_sensor_data;
    QuadrupedCommandData m_cmd_data;



    std::thread m_comm_thread;
};