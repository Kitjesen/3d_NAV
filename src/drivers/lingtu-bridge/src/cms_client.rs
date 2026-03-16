//! CMS gRPC 客户端 — 封装 brainstem CMS 的所有 RPC 调用

use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::net::UdpSocket;
use tonic::transport::Channel;
use tracing::{info, warn, debug};

use crate::quaternion;
use crate::watchdog::Watchdog;

// tonic 生成的 proto 代码
pub mod han_dog {
    tonic::include_proto!("han_dog");
}

use han_dog::cms_client::CmsClient as GrpcCmsClient;

/// CMS 客户端封装
pub struct CmsClient {
    inner: GrpcCmsClient<Channel>,
}

/// 机器人参数
pub struct RobotParams {
    pub robot_type: i32,
}

impl CmsClient {
    /// 连接到 CMS gRPC 服务
    pub async fn connect(endpoint: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let inner = GrpcCmsClient::connect(endpoint.to_string()).await?;
        Ok(Self { inner })
    }

    /// 使能电机
    pub async fn enable(&mut self) -> Result<(), tonic::Status> {
        self.inner.enable(()).await?;
        info!("Motors ENABLED");
        Ok(())
    }

    /// 禁用电机
    pub async fn disable(&mut self) -> Result<(), tonic::Status> {
        self.inner.disable(()).await?;
        info!("Motors DISABLED");
        Ok(())
    }

    /// 行走指令 (归一化后的 [-1, 1])
    pub async fn walk(&mut self, x: f32, y: f32, z: f32) -> Result<(), tonic::Status> {
        let v = han_dog::Vector3 { x, y, z };
        self.inner.walk(v).await?;
        Ok(())
    }

    /// 站立
    pub async fn stand_up(&mut self) -> Result<(), tonic::Status> {
        self.inner.stand_up(()).await?;
        info!("StandUp issued");
        Ok(())
    }

    /// 坐下
    pub async fn sit_down(&mut self) -> Result<(), tonic::Status> {
        self.inner.sit_down(()).await?;
        info!("SitDown issued");
        Ok(())
    }

    /// 获取机器人参数
    pub async fn get_params(&mut self) -> Result<RobotParams, tonic::Status> {
        let resp = self.inner.get_params(()).await?;
        let params = resp.into_inner();
        let robot_type = params
            .robot
            .map(|r| r.r#type)
            .unwrap_or(0);
        Ok(RobotParams { robot_type })
    }

    /// 开始 IMU 流
    pub async fn listen_imu(
        &mut self,
    ) -> Result<tonic::Streaming<han_dog::Imu>, tonic::Status> {
        let resp = self.inner.listen_imu(()).await?;
        Ok(resp.into_inner())
    }

    /// 开始 Joint 流
    pub async fn listen_joint(
        &mut self,
    ) -> Result<tonic::Streaming<han_dog::Joint>, tonic::Status> {
        let resp = self.inner.listen_joint(()).await?;
        Ok(resp.into_inner())
    }
}

// ═══════════════════════════════════════════════════════════
// 并发任务
// ═══════════════════════════════════════════════════════════

/// IMU 流 → UDP 发布
///
/// 数据格式 (48 bytes): [qx, qy, qz, qw, gx, gy, gz, roll, pitch, yaw, vx, vy]
/// 四元数从 Hamilton(w,x,y,z) 转换为 ROS(x,y,z,w)
pub async fn stream_imu(
    client: Arc<Mutex<CmsClient>>,
    udp_port: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    let target = format!("127.0.0.1:{}", udp_port);

    let mut stream = {
        let mut c = client.lock().await;
        c.listen_imu().await?
    };

    info!("IMU stream started → UDP {}", target);
    let mut count: u64 = 0;

    while let Some(imu) = stream.message().await? {
        // Hamilton(w,x,y,z) → ROS(x,y,z,w)
        if let Some(q) = &imu.quaternion {
            let (ros_x, ros_y, ros_z, ros_w) =
                quaternion::hamilton_to_ros(q.w, q.x, q.y, q.z);

            // 角速度
            let (gx, gy, gz) = imu
                .gyroscope
                .as_ref()
                .map(|g| (g.x, g.y, g.z))
                .unwrap_or((0.0, 0.0, 0.0));

            // RPY from quaternion
            let (roll, pitch, yaw) = quaternion::quat_to_euler(ros_x, ros_y, ros_z, ros_w);

            // 打包为 48 bytes (12 x f32)
            let mut buf = [0u8; 48];
            let floats: [f32; 12] = [
                ros_x, ros_y, ros_z, ros_w, // 四元数 (ROS 顺序)
                gx, gy, gz,                  // 角速度
                roll, pitch, yaw,            // 欧拉角
                0.0, 0.0,                    // vx, vy (预留)
            ];
            for (i, &f) in floats.iter().enumerate() {
                buf[i * 4..(i + 1) * 4].copy_from_slice(&f.to_le_bytes());
            }

            let _ = socket.send_to(&buf, &target).await;
        }

        count += 1;
        if count % 500 == 0 {
            debug!("IMU: {} frames sent", count);
        }
    }

    warn!("IMU stream ended");
    Ok(())
}

/// Joint 流 → UDP 发布
///
/// 数据格式: [16 pos, 16 vel, 16 torque] = 192 bytes (48 x f32)
pub async fn stream_joints(
    client: Arc<Mutex<CmsClient>>,
    udp_port: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind("0.0.0.0:0").await?;
    let target = format!("127.0.0.1:{}", udp_port);

    let mut stream = {
        let mut c = client.lock().await;
        c.listen_joint().await?
    };

    info!("Joint stream started → UDP {}", target);
    let mut count: u64 = 0;

    while let Some(joint) = stream.message().await? {
        if let Some(han_dog::joint::Data::AllJoints(all)) = joint.data {
            let pos = &all.position.as_ref().map(|m| &m.values[..]).unwrap_or(&[]);
            let vel = &all.velocity.as_ref().map(|m| &m.values[..]).unwrap_or(&[]);
            let tor = &all.torque.as_ref().map(|m| &m.values[..]).unwrap_or(&[]);

            // 打包 192 bytes (48 x f32)
            let mut buf = [0u8; 192];
            for i in 0..16 {
                let p = pos.get(i).copied().unwrap_or(0.0);
                let v = vel.get(i).copied().unwrap_or(0.0);
                let t = tor.get(i).copied().unwrap_or(0.0);
                buf[i * 4..(i + 1) * 4].copy_from_slice(&p.to_le_bytes());
                buf[(16 + i) * 4..(17 + i) * 4].copy_from_slice(&v.to_le_bytes());
                buf[(32 + i) * 4..(33 + i) * 4].copy_from_slice(&t.to_le_bytes());
            }

            let _ = socket.send_to(&buf, &target).await;

            count += 1;
            if count % 500 == 0 {
                debug!("Joint: {} frames sent", count);
            }
        }
    }

    warn!("Joint stream ended");
    Ok(())
}

/// cmd_vel UDP 接收 → Walk() gRPC 调用
///
/// 接收格式: [vx, vy, wz] = 12 bytes (3 x f32)
pub async fn cmd_vel_loop(
    client: Arc<Mutex<CmsClient>>,
    watchdog: Watchdog,
    udp_port: u16,
    max_linear: f32,
    max_angular: f32,
) -> Result<(), Box<dyn std::error::Error>> {
    let socket = UdpSocket::bind(format!("0.0.0.0:{}", udp_port)).await?;
    info!("cmd_vel listener on UDP {} | max_lin={} max_ang={}", udp_port, max_linear, max_angular);

    let mut buf = [0u8; 12];
    let mut count: u64 = 0;

    loop {
        let (len, _addr) = socket.recv_from(&mut buf).await?;
        if len < 12 {
            continue;
        }

        // 解析 [vx, vy, wz]
        let vx = f32::from_le_bytes(buf[0..4].try_into().unwrap());
        let vy = f32::from_le_bytes(buf[4..8].try_into().unwrap());
        let wz = f32::from_le_bytes(buf[8..12].try_into().unwrap());

        // 归一化到 [-1, 1]
        let walk_x = clamp(vx / max_linear, -1.0, 1.0);
        let walk_y = clamp(vy / max_linear, -1.0, 1.0);
        let walk_z = clamp(wz / max_angular, -1.0, 1.0);

        // 有限值检查
        if !walk_x.is_finite() || !walk_y.is_finite() || !walk_z.is_finite() {
            warn!("cmd_vel NaN/Inf, skipping");
            continue;
        }

        // 喂看门狗
        watchdog.feed();

        // 发送 Walk
        {
            let mut c = client.lock().await;
            if let Err(e) = c.walk(walk_x, walk_y, walk_z).await {
                warn!("Walk failed: {}", e);
            }
        }

        count += 1;
        if count % 100 == 0 {
            debug!("cmd_vel: {} cmds, last=({:.2},{:.2},{:.2})", count, walk_x, walk_y, walk_z);
        }
    }
}

#[inline]
fn clamp(v: f32, min: f32, max: f32) -> f32 {
    v.max(min).min(max)
}
