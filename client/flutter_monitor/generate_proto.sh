#!/bin/bash
# 生成 Dart gRPC 代码脚本

set -e

echo "Generating Dart gRPC code from proto files..."

# 检查 protoc
if ! command -v protoc &> /dev/null; then
    echo "Error: protoc not found. Please install Protocol Buffers compiler."
    exit 1
fi

# 检查 protoc-gen-dart
if ! command -v protoc-gen-dart &> /dev/null; then
    echo "Error: protoc-gen-dart not found."
    echo "Install with: dart pub global activate protoc_plugin"
    echo "Then add to PATH: export PATH=\"\$PATH:\$HOME/.pub-cache/bin\""
    exit 1
fi

# 清理旧文件
rm -rf lib/generated
mkdir -p lib/generated

# 生成代码
protoc --dart_out=grpc:lib/generated \
  -I proto \
  proto/common.proto \
  proto/system.proto \
  proto/control.proto \
  proto/telemetry.proto \
  proto/data.proto

echo "✓ Code generation completed!"
echo "Generated files in lib/generated/"
ls -lh lib/generated/*.dart
