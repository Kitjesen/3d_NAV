#pragma once
#include "ieskf.h"
#include "commons.h"

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

private:
    void checkIMUStationary(const Vec<IMUData> &batch);

    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;

    // ZUPT static detection state
    int m_static_frame_count = 0;
};