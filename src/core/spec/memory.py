"""Protocol interfaces for spatial and episodic memory (L3 / L4).

Two complementary memory abstractions used by the semantic planner:

* **SpatialMemory** -- topological graph of visited locations with
  text-based retrieval (backed by ``TopologicalMemory``).
* **EpisodicMemoryLike** -- FIFO record store with LLM-friendly
  formatting (backed by ``EpisodicMemory``).
"""

from __future__ import annotations

from typing import Any, List, Optional, Protocol, runtime_checkable


@runtime_checkable
class SpatialMemory(Protocol):
    """Spatial / topological memory interface.

    Implementations: TopologicalMemory (FSR-VLN viewpoint edges,
    VLingMem region summaries).
    """

    def update_position(
        self,
        position: Any,
        visible_labels: list[str] | None = None,
    ) -> None:
        """Record the robot's current position and visible objects."""
        ...

    def query_by_text(self, text: str, top_k: int = 5) -> list:
        """Retrieve the *top_k* memory nodes most relevant to *text*."""
        ...


@runtime_checkable
class EpisodicMemoryLike(Protocol):
    """Episodic memory interface (ReMEmbR-style FIFO).

    Implementations: EpisodicMemory (500-record spatiotemporal store).
    """

    def add(self, position: Any, label: str) -> None:
        """Append an observation record at *position* with *label*."""
        ...

    def format_for_llm(self) -> str:
        """Render the memory contents as an LLM-consumable string."""
        ...
