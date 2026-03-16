//! lingtu-bridge — 高性能 Rust 通信桥
//!
//! 替代 han_dog_bridge.py，用 Rust 实现:
//!   ROS2 /nav/cmd_vel → gRPC Walk() → brainstem CMS :13145
//!   CMS ListenImu()   → /nav/dog_odometry
//!   CMS ListenJoint() → /robot_state
//!   看门狗: 200ms 无 cmd_vel → Walk(0,0,0)

mod cms_client;
mod watchdog;
mod quaternion;

use clap::Parser;
use tracing::{info, warn, error};

/// LingTu Bridge — brainstem CMS gRPC 通信桥
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// brainstem CMS 主机地址
    #[arg(long, default_value = "127.0.0.1")]
    dog_host: String,

    /// brainstem CMS 端口
    #[arg(long, default_value_t = 13145)]
    dog_port: u16,

    /// 最大线速度 (m/s)，用于归一化
    #[arg(long, default_value_t = 1.0)]
    max_linear_speed: f32,

    /// 最大角速度 (rad/s)，用于归一化
    #[arg(long, default_value_t = 1.0)]
    max_angular_speed: f32,

    /// cmd_vel 看门狗超时 (ms)
    #[arg(long, default_value_t = 200)]
    watchdog_timeout_ms: u64,

    /// 启动时自动 Enable + StandUp
    #[arg(long, default_value_t = true)]
    auto_standup: bool,

    /// UDP 监听端口 (接收 cmd_vel)
    #[arg(long, default_value_t = 9870)]
    cmd_port: u16,

    /// UDP 发布端口 (发送 IMU/Joint)
    #[arg(long, default_value_t = 9871)]
    pub_port: u16,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 初始化日志
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "lingtu_bridge=info".into()),
        )
        .init();

    let args = Args::parse();
    info!(
        "lingtu-bridge v{} | CMS {}:{} | watchdog {}ms",
        env!("CARGO_PKG_VERSION"),
        args.dog_host,
        args.dog_port,
        args.watchdog_timeout_ms,
    );

    // 创建 CMS 客户端
    let endpoint = format!("http://{}:{}", args.dog_host, args.dog_port);
    let mut client = cms_client::CmsClient::connect(&endpoint).await?;

    // 获取机器人参数
    let params = client.get_params().await?;
    info!("Connected to CMS | robot_type={}", params.robot_type);

    // 自动 Enable + StandUp
    if args.auto_standup {
        info!("Auto enable + standup...");
        client.enable().await?;
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        client.stand_up().await?;
        info!("StandUp issued, waiting 3s for transition...");
        tokio::time::sleep(std::time::Duration::from_secs(3)).await;
    }

    // 启动看门狗
    let watchdog = watchdog::Watchdog::new(
        std::time::Duration::from_millis(args.watchdog_timeout_ms),
    );

    // 启动并发任务
    let client_arc = std::sync::Arc::new(tokio::sync::Mutex::new(client));

    // Task 1: IMU 流
    let imu_client = client_arc.clone();
    let pub_port = args.pub_port;
    let imu_handle = tokio::spawn(async move {
        if let Err(e) = cms_client::stream_imu(imu_client, pub_port).await {
            error!("IMU stream error: {}", e);
        }
    });

    // Task 2: Joint 流
    let joint_client = client_arc.clone();
    let joint_handle = tokio::spawn(async move {
        if let Err(e) = cms_client::stream_joints(joint_client, pub_port + 1).await {
            error!("Joint stream error: {}", e);
        }
    });

    // Task 3: cmd_vel 接收 + Walk 发送 + 看门狗
    let walk_client = client_arc.clone();
    let wd = watchdog.clone();
    let max_lin = args.max_linear_speed;
    let max_ang = args.max_angular_speed;
    let cmd_port = args.cmd_port;
    let cmd_handle = tokio::spawn(async move {
        if let Err(e) = cms_client::cmd_vel_loop(
            walk_client, wd, cmd_port, max_lin, max_ang,
        ).await {
            error!("cmd_vel loop error: {}", e);
        }
    });

    // Task 4: 看门狗零速度发送
    let wd_client = client_arc.clone();
    let wd2 = watchdog.clone();
    let wd_handle = tokio::spawn(async move {
        watchdog::watchdog_loop(wd2, wd_client).await;
    });

    info!("All tasks started. Press Ctrl+C to stop.");

    // 等待 Ctrl+C
    tokio::signal::ctrl_c().await?;
    warn!("Shutting down...");

    // 发送零速度
    {
        let mut c = client_arc.lock().await;
        let _ = c.walk(0.0, 0.0, 0.0).await;
    }

    // 取消所有任务
    imu_handle.abort();
    joint_handle.abort();
    cmd_handle.abort();
    wd_handle.abort();

    info!("lingtu-bridge stopped.");
    Ok(())
}
