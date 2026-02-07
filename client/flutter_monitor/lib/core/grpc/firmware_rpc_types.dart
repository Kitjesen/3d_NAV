// Re-export OTA types from generated proto.
// All OTA types are defined in data.proto.

export 'package:robot_proto/src/data.pb.dart'
    show
        ApplyUpdateRequest,
        ApplyUpdateResponse,
        GetInstalledVersionsRequest,
        GetInstalledVersionsResponse,
        RollbackRequest,
        RollbackResponse,
        DownloadFromUrlRequest,
        OtaProgress,
        CheckUpdateReadinessRequest,
        CheckUpdateReadinessResponse,
        ReadinessCheck,
        OtaArtifact,
        OtaCategory,
        OtaApplyAction,
        OtaUpdateStatus,
        InstalledArtifact,
        RollbackEntry;
