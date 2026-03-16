//! 四元数工具 — Hamilton ↔ ROS 转换 + 欧拉角
//!
//! brainstem CMS proto: Hamilton 约定 (w, x, y, z)
//! ROS2 Odometry msg:   ROS 约定 (x, y, z, w)

/// Hamilton(w,x,y,z) → ROS(x,y,z,w)
#[inline]
pub fn hamilton_to_ros(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32, f32) {
    // 归一化
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    if norm < 1e-7 {
        return (0.0, 0.0, 0.0, 1.0); // identity
    }
    let inv = 1.0 / norm;
    (x * inv, y * inv, z * inv, w * inv)
}

/// ROS 四元数(x,y,z,w) → 欧拉角(roll, pitch, yaw) in radians
#[inline]
pub fn quat_to_euler(x: f32, y: f32, z: f32, w: f32) -> (f32, f32, f32) {
    // Roll (x-axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(sinp) // gimbal lock
    } else {
        sinp.asin()
    };

    // Yaw (z-axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_quaternion() {
        // Hamilton identity: (1, 0, 0, 0) → ROS: (0, 0, 0, 1)
        let (x, y, z, w) = hamilton_to_ros(1.0, 0.0, 0.0, 0.0);
        assert!((w - 1.0).abs() < 1e-6);
        assert!(x.abs() < 1e-6);
        assert!(y.abs() < 1e-6);
        assert!(z.abs() < 1e-6);
    }

    #[test]
    fn test_euler_identity() {
        let (roll, pitch, yaw) = quat_to_euler(0.0, 0.0, 0.0, 1.0);
        assert!(roll.abs() < 1e-6);
        assert!(pitch.abs() < 1e-6);
        assert!(yaw.abs() < 1e-6);
    }

    #[test]
    fn test_90deg_yaw() {
        // 90° yaw: ROS quat (0, 0, sin(45°), cos(45°))
        let s = std::f32::consts::FRAC_PI_4.sin();
        let c = std::f32::consts::FRAC_PI_4.cos();
        let (roll, pitch, yaw) = quat_to_euler(0.0, 0.0, s, c);
        assert!(roll.abs() < 1e-5);
        assert!(pitch.abs() < 1e-5);
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn test_hamilton_to_ros_roundtrip() {
        // Hamilton (w=0.707, x=0, y=0, z=0.707) = 90° yaw
        let (rx, ry, rz, rw) = hamilton_to_ros(0.7071068, 0.0, 0.0, 0.7071068);
        let (_, _, yaw) = quat_to_euler(rx, ry, rz, rw);
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }
}
