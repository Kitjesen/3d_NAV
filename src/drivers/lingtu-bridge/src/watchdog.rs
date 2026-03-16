//! 看门狗 — 200ms 无 cmd_vel 自动发送零速度
//!
//! 原 Python 版用 threading.Timer，精度 ~10ms。
//! Rust 版用 tokio::time，精度 ~1ms。

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use tokio::sync::Mutex;
use tracing::{warn, debug};

use crate::cms_client::CmsClient;

/// 看门狗状态 (lock-free)
#[derive(Clone)]
pub struct Watchdog {
    /// 最后一次 feed 的时间戳 (nanos since epoch)
    last_feed: Arc<AtomicU64>,
    /// 超时时长
    timeout: Duration,
    /// 启动时间基准
    start: Instant,
}

impl Watchdog {
    pub fn new(timeout: Duration) -> Self {
        let now = Instant::now();
        let wd = Self {
            last_feed: Arc::new(AtomicU64::new(0)),
            timeout,
            start: now,
        };
        wd.feed(); // 初始 feed
        wd
    }

    /// 喂狗 — 每次收到 cmd_vel 调用
    pub fn feed(&self) {
        let elapsed = self.start.elapsed().as_nanos() as u64;
        self.last_feed.store(elapsed, Ordering::Relaxed);
    }

    /// 检查是否超时
    pub fn is_expired(&self) -> bool {
        let last = self.last_feed.load(Ordering::Relaxed);
        let now = self.start.elapsed().as_nanos() as u64;
        let diff = Duration::from_nanos(now.saturating_sub(last));
        diff > self.timeout
    }
}

/// 看门狗循环 — 定期检查是否超时，超时则发零速度
pub async fn watchdog_loop(
    watchdog: Watchdog,
    client: Arc<Mutex<CmsClient>>,
) {
    let check_interval = watchdog.timeout / 2; // 以超时的一半频率检查
    let mut was_expired = false;
    let mut zero_sent = false;

    loop {
        tokio::time::sleep(check_interval).await;

        if watchdog.is_expired() {
            if !was_expired {
                warn!(
                    "Watchdog: cmd_vel timeout ({}ms), sending zero velocity",
                    watchdog.timeout.as_millis()
                );
                was_expired = true;
                zero_sent = false;
            }

            // 只发一次零速度，不持续发
            if !zero_sent {
                let mut c = client.lock().await;
                match c.walk(0.0, 0.0, 0.0).await {
                    Ok(_) => {
                        debug!("Watchdog: zero velocity sent");
                        zero_sent = true;
                    }
                    Err(e) => warn!("Watchdog: Walk(0,0,0) failed: {}", e),
                }
            }
        } else {
            if was_expired {
                debug!("Watchdog: cmd_vel resumed");
            }
            was_expired = false;
            zero_sent = false;
        }
    }
}
