#include "Planner/Gait.hpp"
#include <iostream>

Gait::Gait() : m_name("gait"), m_gait_type(STANCE)
{
    InitClass();
}

Gait::Gait(GAIT_TYPE gait_type) : m_name("gait"), m_gait_type(gait_type)
{
    InitClass();
}

Gait::Gait(std::string name, GAIT_TYPE gait_type) : m_name(name), m_gait_type(gait_type)
{
    InitClass();
}

void Gait::SetGaitPeriod(const float &t)
{
    m_gait_period = t;
}

float Gait::GetStepHeight()
{
    return m_step_height;
}

void Gait::SetStepHeight(const float step_height)
{
    m_step_height = step_height;
}

void Gait::SetParams(float gait_period, float t0, float step_height, Eigen::Vector4d phi_thresh, Eigen::Vector4d phi_offset)
{
    m_gait_period = gait_period;
    m_t0 = t0;
    m_phi_thresh = phi_thresh;
    m_phi_offset = phi_offset;
    m_step_height = step_height;
}

void Gait::ShowParams()
{
    std::cout << "Gait Type: " << m_gait_type << "\n";
    std::cout << "Starting Point (t0): " << m_t0 << "\n";
    std::cout << "Gait Period (t_gp): " << m_gait_period << "\n";
    std::cout << "Phase Thresholds (phi_threshold): " << m_phi_thresh.transpose() << "\n";
    std::cout << "Phase Offsets (phi_offset): " << m_phi_offset.transpose() << "\n";
    std::cout << "Step Height (h): " << m_step_height << "\n";
}

Eigen::Array4i Gait::GetScheduledContact(const float &t)
{
    for (int i = 0; i < 4; ++i)
    {
        float phi = GetStridePhase(t, i);
        m_cs_ref_data(i) = phi > m_phi_thresh(i) ? 0 : 1;
    }

    return m_cs_ref_data;
}

float Gait::GetStridePhase(const float &t, const int &leg_id)
{
    return fmod((t - m_t0) / m_gait_period + m_phi_offset(leg_id), 1);
}

float Gait::GetSwingState(const float &t, const int &leg_id)
{
    float phi = GetStridePhase(t, leg_id);

    if (m_cs_ref_data(leg_id))
    {
        // leg in stance phase, set swing state as -1
        return -1;
    }
    else
    {
        // leg in swing phase, compute the swing state
        return (phi - m_phi_thresh(leg_id)) / (1 - m_phi_thresh(leg_id));
    }
}

float Gait::GetStanceState(const float &t, const int &leg_id)
{
    float phi = GetStridePhase(t, leg_id);

    if (m_cs_ref_data(leg_id))
    {
        // leg in stance phase, compute the stance state
        return phi / m_phi_thresh(leg_id);
    }
    else
    {
        // leg in swing phase, set the stance state as -1
        return -1;
    }
}

void Gait::UpdateStrideCount(int leg_id)
{
    m_n_stride(leg_id)++;
}

int Gait::GetStrideCount(int leg_id)
{
    return m_n_stride(leg_id);
}

void Gait::ResetStrideCount()
{
    m_n_stride = Eigen::Array4i::Zero();
}

float Gait::GetStanceTime(const int &leg_id)
{
    return m_gait_period * m_phi_thresh(leg_id);
}

float Gait::GetSwingTime(const int &leg_id)
{
    return m_gait_period * (1.0 - m_phi_thresh(leg_id));
}

Gait::GAIT_TYPE Gait::GetGaitType()
{
    return m_gait_type;
}

void Gait::InitClass()
{
    // meshin: Initialize these based on GAIT_TYPE
    if (m_gait_type == TROT) {
        m_t0 = 0;
        m_phi_thresh = 0.5 * Eigen::Vector4d::Ones();
        m_phi_offset << 0.5f, 0.0f, 0.5f, 0.0f;
        m_cs_ref_data = Eigen::Array4i::Ones();
        m_gait_period = 0.5;
        m_step_height = 0.06;
    } else if (m_gait_type == STANCE) {
        m_t0 = 0;
        m_phi_thresh = Eigen::Vector4d::Ones();
        m_phi_offset = Eigen::Vector4d::Zero();
        m_cs_ref_data = Eigen::Array4i::Ones();
        m_gait_period = 1;
        m_step_height = 0.08;
    } else {
        m_t0 = 0;
        m_phi_thresh = Eigen::Vector4d::Ones();
        m_phi_offset = Eigen::Vector4d::Zero();
        m_cs_ref_data = Eigen::Array4i::Ones();
        m_gait_period = 1;
        m_step_height = 0.08;
    }
}