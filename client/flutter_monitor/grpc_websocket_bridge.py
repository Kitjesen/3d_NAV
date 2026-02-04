#!/usr/bin/env python3
"""
gRPC WebSocket Bridge
将 gRPC 服务桥接到 WebSocket，使浏览器可以访问
"""

import asyncio
import json
import grpc
import sys
import os
from aiohttp import web
import aiohttp_cors

# 添加 generated Python proto 路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/remote_monitoring/build/generated'))

try:
    from telemetry_pb2 import FastStateRequest
    from telemetry_pb2_grpc import TelemetryServiceStub
    from system_pb2_grpc import SystemServiceStub
    from google.protobuf.empty_pb2 import Empty
    from google.protobuf.json_format import MessageToDict
except ImportError:
    print("⚠️  警告：无法导入 protobuf 生成的 Python 代码")
    print("   请先构建 remote_monitoring 项目")
    sys.exit(1)


class GrpcWebSocketBridge:
    def __init__(self, grpc_host='192.168.66.190', grpc_port=50051):
        self.grpc_host = grpc_host
        self.grpc_port = grpc_port
        self.channel = None
        self.telemetry_client = None
        self.system_client = None
    
    async def connect_grpc(self):
        """连接到 gRPC 服务"""
        self.channel = grpc.aio.insecure_channel(f'{self.grpc_host}:{self.grpc_port}')
        self.telemetry_client = TelemetryServiceStub(self.channel)
        self.system_client = SystemServiceStub(self.channel)
        print(f"✓ 已连接到 gRPC 服务: {self.grpc_host}:{self.grpc_port}")
    
    async def get_robot_info(self):
        """获取机器人信息"""
        response = await self.system_client.GetRobotInfo(Empty())
        return MessageToDict(response)
    
    async def get_capabilities(self):
        """获取机器人能力"""
        response = await self.system_client.GetCapabilities(Empty())
        return MessageToDict(response)
    
    async def stream_fast_state(self, desired_hz=10.0):
        """订阅快速状态流"""
        request = FastStateRequest(desired_hz=desired_hz)
        async for state in self.telemetry_client.StreamFastState(request):
            yield MessageToDict(state)


bridge = GrpcWebSocketBridge()


async def websocket_handler(request):
    """WebSocket 处理器"""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    print(f"✓ WebSocket 客户端已连接: {request.remote}")
    
    try:
        # 发送欢迎消息
        await ws.send_json({
            'type': 'connected',
            'message': 'Connected to gRPC bridge'
        })
        
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                command = data.get('command')
                
                if command == 'get_robot_info':
                    info = await bridge.get_robot_info()
                    await ws.send_json({'type': 'robot_info', 'data': info})
                
                elif command == 'get_capabilities':
                    caps = await bridge.get_capabilities()
                    await ws.send_json({'type': 'capabilities', 'data': caps})
                
                elif command == 'start_stream':
                    desired_hz = data.get('desired_hz', 10.0)
                    await ws.send_json({'type': 'stream_started'})
                    
                    # 开始流式传输
                    async for state in bridge.stream_fast_state(desired_hz):
                        await ws.send_json({'type': 'fast_state', 'data': state})
                
                elif command == 'ping':
                    await ws.send_json({'type': 'pong'})
            
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print(f'❌ WebSocket 错误: {ws.exception()}')
    
    except Exception as e:
        print(f'❌ 错误: {e}')
    
    finally:
        print(f'✗ WebSocket 客户端已断开: {request.remote}')
    
    return ws


async def http_health_check(request):
    """HTTP 健康检查"""
    return web.json_response({
        'status': 'ok',
        'grpc_endpoint': f'{bridge.grpc_host}:{bridge.grpc_port}'
    })


async def on_startup(app):
    """启动时连接 gRPC"""
    await bridge.connect_grpc()


async def on_cleanup(app):
    """清理时关闭连接"""
    if bridge.channel:
        await bridge.channel.close()


def main():
    print("🚀 启动 gRPC WebSocket Bridge")
    print("=" * 60)
    
    app = web.Application()
    
    # 配置 CORS（允许浏览器跨域访问）
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*"
        )
    })
    
    # 添加路由
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/health', http_health_check)
    
    # 配置 CORS
    for route in list(app.router.routes()):
        cors.add(route)
    
    # 启动和清理钩子
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    
    print("✓ WebSocket 服务地址: ws://0.0.0.0:8081/ws")
    print("✓ HTTP 健康检查: http://0.0.0.0:8081/health")
    print("=" * 60)
    
    web.run_app(app, host='0.0.0.0', port=8081)


if __name__ == '__main__':
    main()
