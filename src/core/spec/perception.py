"""Protocol interfaces for the Perception layer (L3).

Defines structural contracts that any detector, encoder, or tracker backend
must satisfy.  These are *not* base classes -- they use ``typing.Protocol``
so that existing implementations are automatically compatible without
explicit inheritance (structural subtyping).
"""

from __future__ import annotations

from typing import Any, List, Optional, Protocol, runtime_checkable


@runtime_checkable
class Detector(Protocol):
    """Object detector interface.

    Any detector backend (YOLO-World, Grounding DINO, BPU, etc.) must
    expose at least these three methods.
    """

    def detect(self, image: Any, classes: list[str] | None = None) -> list:
        """Run detection on an image, optionally restricting to *classes*."""
        ...

    def load_model(self) -> None:
        """Load / initialise the underlying model weights."""
        ...

    def shutdown(self) -> None:
        """Release model resources."""
        ...


@runtime_checkable
class Encoder(Protocol):
    """Feature encoder interface (CLIP family).

    Implementations: CLIPEncoder, MobileCLIPEncoder, etc.
    """

    def encode_image(self, image: Any) -> Any:
        """Encode an image into a feature vector."""
        ...

    def encode_text(self, text: str) -> Any:
        """Encode a text string into a feature vector."""
        ...

    def load_model(self) -> None:
        """Load / initialise the encoder model."""
        ...


@runtime_checkable
class Tracker(Protocol):
    """Instance tracker interface.

    Maintains cross-frame object identity and builds the scene graph.
    """

    def update(self, detections: Any, **kwargs: Any) -> None:
        """Ingest a new frame of detections."""
        ...

    def get_scene_graph_json(self) -> str:
        """Export the current scene graph as a JSON string."""
        ...

    def clear(self) -> None:
        """Reset all tracked instances."""
        ...
