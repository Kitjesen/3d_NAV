"""App and web client routes for GatewayModule."""

from __future__ import annotations

from gateway.schemas import AppBootstrapResponse, AppCapabilitiesResponse
from gateway.services.app_bootstrap import build_app_bootstrap, build_app_capabilities


def register_app_routes(app, gw) -> None:
    @app.get(
        "/api/v1/app/bootstrap",
        summary="App/Web bootstrap snapshot",
        response_model=AppBootstrapResponse,
    )
    async def app_bootstrap():
        return build_app_bootstrap(gw)

    @app.get(
        "/api/v1/app/capabilities",
        summary="App/Web API capability manifest",
        response_model=AppCapabilitiesResponse,
    )
    async def app_capabilities():
        return build_app_capabilities(gw)
