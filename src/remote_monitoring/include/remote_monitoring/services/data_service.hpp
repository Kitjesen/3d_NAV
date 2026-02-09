#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "data.grpc.pb.h"
#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"

#ifdef WEBRTC_ENABLED
#include "remote_monitoring/webrtc_bridge.hpp"
#endif

namespace remote_monitoring {
namespace core {
class HealthMonitor;
}

namespace services {

class DataServiceImpl final : public robot::v1::DataService::Service {
public:
  explicit DataServiceImpl(rclcpp::Node *node);

  /// 注入 HealthMonitor 用于安装后健康检查 (由 GrpcGateway 调用)
  void SetHealthMonitor(std::shared_ptr<core::HealthMonitor> monitor) {
    health_monitor_ = std::move(monitor);
  }

  grpc::Status
  ListResources(grpc::ServerContext *context,
                const google::protobuf::Empty *request,
                robot::v1::ListResourcesResponse *response) override;

  grpc::Status
  Subscribe(grpc::ServerContext *context,
            const robot::v1::SubscribeRequest *request,
            grpc::ServerWriter<robot::v1::DataChunk> *writer) override;

  grpc::Status Unsubscribe(grpc::ServerContext *context,
                           const robot::v1::UnsubscribeRequest *request,
                           robot::v1::UnsubscribeResponse *response) override;

  grpc::Status
  DownloadFile(grpc::ServerContext *context,
               const robot::v1::DownloadFileRequest *request,
               grpc::ServerWriter<robot::v1::FileChunk> *writer) override;

  // 文件上传 (OTA 部署)
  grpc::Status
  UploadFile(grpc::ServerContext *context,
             grpc::ServerReader<robot::v1::UploadFileChunk> *reader,
             robot::v1::UploadFileResponse *response) override;

  // 列出远程目录文件
  grpc::Status
  ListRemoteFiles(grpc::ServerContext *context,
                  const robot::v1::ListRemoteFilesRequest *request,
                  robot::v1::ListRemoteFilesResponse *response) override;

  // 删除远程文件
  grpc::Status
  DeleteRemoteFile(grpc::ServerContext *context,
                   const robot::v1::DeleteRemoteFileRequest *request,
                   robot::v1::DeleteRemoteFileResponse *response) override;

  // OTA 更新管理
  grpc::Status
  ApplyUpdate(grpc::ServerContext *context,
              const robot::v1::ApplyUpdateRequest *request,
              robot::v1::ApplyUpdateResponse *response) override;

  grpc::Status
  GetInstalledVersions(grpc::ServerContext *context,
                       const robot::v1::GetInstalledVersionsRequest *request,
                       robot::v1::GetInstalledVersionsResponse *response) override;

  grpc::Status
  Rollback(grpc::ServerContext *context,
           const robot::v1::RollbackRequest *request,
           robot::v1::RollbackResponse *response) override;

  // 机器人直接从 URL 下载 (流式进度)
  grpc::Status
  DownloadFromUrl(grpc::ServerContext *context,
                  const robot::v1::DownloadFromUrlRequest *request,
                  grpc::ServerWriter<robot::v1::OtaProgress> *writer) override;

  // 安装前预检查
  grpc::Status
  CheckUpdateReadiness(grpc::ServerContext *context,
                       const robot::v1::CheckUpdateReadinessRequest *request,
                       robot::v1::CheckUpdateReadinessResponse *response) override;

  // 升级历史查询
  grpc::Status
  GetUpgradeHistory(grpc::ServerContext *context,
                    const robot::v1::GetUpgradeHistoryRequest *request,
                    robot::v1::GetUpgradeHistoryResponse *response) override;

  // 版本一致性校验
  grpc::Status
  ValidateSystemVersion(grpc::ServerContext *context,
                        const robot::v1::ValidateSystemVersionRequest *request,
                        robot::v1::ValidateSystemVersionResponse *response) override;

  grpc::Status StartCamera(grpc::ServerContext *context,
                           const robot::v1::StartCameraRequest *request,
                           robot::v1::StartCameraResponse *response) override;

  grpc::Status StopCamera(grpc::ServerContext *context,
                          const robot::v1::StopCameraRequest *request,
                          robot::v1::StopCameraResponse *response) override;

  // WebRTC 双向信令流
  grpc::Status WebRTCSignaling(
      grpc::ServerContext *context,
      grpc::ServerReaderWriter<robot::v1::WebRTCSignal,
                               robot::v1::WebRTCSignal> *stream) override;

private:
  bool IsCompressionSupported(robot::v1::CompressionType compression) const;
  double ResolveFrequency(const robot::v1::SubscribeRequest *request) const;
  std::string ResolveTopic(const robot::v1::SubscribeRequest *request) const;

  bool RegisterSubscription(
      const std::string &subscription_id,
      std::shared_ptr<std::atomic_bool> *cancel_flag);
  void RemoveSubscription(const std::string &subscription_id);

  struct WebrtcSessionInfo {
    std::string camera_id;
    std::string topic;
    std::string offer_path;
    std::string ice_path;
  };

  // WebRTC 双向信令会话状态
  struct WebRTCSession {
    std::string session_id;
    robot::v1::WebRTCSessionConfig config;
    std::queue<robot::v1::WebRTCSignal> outgoing_signals;
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic_bool active{true};
    std::atomic_bool peer_connected{false};
    std::string local_sdp;
    std::string remote_sdp;
    std::vector<std::string> local_ice_candidates;
    std::vector<std::string> remote_ice_candidates;
  };

  // OTA 管理私有方法
  bool LoadInstalledManifest();
  bool SaveInstalledManifest();
  std::string ComputeSHA256(const std::string &file_path);
  bool BackupArtifact(const std::string &name, const std::string &current_path,
                      std::string *backup_path);
  uint64_t GetDiskFreeBytes(const std::string &path);
  int GetBatteryPercent();

  // Phase 1.1: 系统版本管理
  std::string LoadSystemVersionJson();
  void SaveSystemVersionJson();

  // Phase 1.2: Ed25519 设备端验签
  bool VerifyEd25519Signature(const std::string &message_hex,
                              const std::string &signature_hex);

  // Phase 1.3: 安装后健康检查
  bool PostInstallHealthCheck(robot::v1::OtaSafetyLevel safety_level,
                              std::string *failure_reason);

  // Phase 1.4: semver 比较 (-1, 0, +1)
  static int CompareSemver(const std::string &a, const std::string &b);

  // Phase 2.1: 持久升级历史
  void AppendUpgradeHistory(const std::string &action,
                            const std::string &artifact_name,
                            const std::string &from_version,
                            const std::string &to_version,
                            const std::string &status,
                            robot::v1::OtaFailureCode failure_code,
                            const std::string &failure_reason,
                            uint64_t duration_ms,
                            const std::string &health_check);

  rclcpp::Node *node_;
  std::string apply_firmware_script_;
  std::string ota_manifest_path_;       // 本地已安装 manifest 路径
  std::string ota_backup_dir_;          // 备份目录
  std::string ota_public_key_path_;     // Ed25519 公钥路径
  std::string ota_history_path_;        // 升级历史日志路径
  std::string system_version_path_;     // system_version.json 路径
  std::string robot_id_;
  std::string hw_id_;
  std::string system_version_;
  std::shared_ptr<core::HealthMonitor> health_monitor_; // 安装后健康检查
  // 已安装制品 (name -> InstalledArtifact proto)
  std::unordered_map<std::string, robot::v1::InstalledArtifact> installed_artifacts_;
  // 可回滚条目 (name -> RollbackEntry proto)
  std::unordered_map<std::string, robot::v1::RollbackEntry> rollback_entries_;
  mutable std::mutex ota_mutex_;
  std::string camera_topic_;
  std::string camera_fallback_topic_;
  std::string map_topic_;
  std::string pointcloud_topic_;
  std::string terrain_topic_;
  std::string file_root_;
  bool webrtc_enabled_{false};
  int webrtc_offer_timeout_ms_{3000};
  std::string webrtc_start_command_;
  std::string webrtc_stop_command_;
  std::string webrtc_offer_path_;
  std::string webrtc_ice_path_;
  mutable std::mutex subscriptions_mutex_;
  std::unordered_map<std::string, std::shared_ptr<std::atomic_bool>>
      active_subscriptions_;
  mutable std::mutex camera_sessions_mutex_;
  std::unordered_set<std::string> active_camera_sessions_;
  mutable std::mutex webrtc_mutex_;
  std::unordered_map<std::string, WebrtcSessionInfo> webrtc_sessions_;
  
  // WebRTC 双向信令会话管理
  mutable std::mutex webrtc_signaling_mutex_;
  std::unordered_map<std::string, std::shared_ptr<WebRTCSession>> webrtc_signaling_sessions_;
  
  // WebRTC 信令处理辅助方法
  std::shared_ptr<WebRTCSession> GetOrCreateWebRTCSession(const std::string &session_id);
  void RemoveWebRTCSession(const std::string &session_id);
  void HandleWebRTCOffer(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);
  void HandleWebRTCAnswer(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);
  void HandleWebRTCIceCandidate(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);

#ifdef WEBRTC_ENABLED
  // WebRTC 媒体桥接器
  std::unique_ptr<WebRTCBridge> webrtc_bridge_;
  void InitializeWebRTCBridge();
  void SendWebRTCSignalToSession(const std::string &session_id, robot::v1::WebRTCSignal signal);
#endif
};

} // namespace services
} // namespace remote_monitoring
