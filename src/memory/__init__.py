"""lingtu.memory -- unified memory layer.

Spatial memory (where), knowledge (what), storage (how to persist),
scheduling (when to look), logging (what happened).

Usage::

    from memory.spatial.topological import TopologicalMemory
    from memory.knowledge.knowledge_graph import IndustrialKnowledgeGraph
    from memory.storage.sqlite_store import SQLiteSceneStore
    from memory.modules import TopologicalMemoryModule
"""

# Re-export core classes for convenience
from .modules.episodic_module import EpisodicMemoryModule
from .modules.tagged_locations_module import TaggedLocationsModule
from .modules.topological_module import TopologicalMemoryModule
from .spatial.episodic import EpisodicMemory
from .spatial.tagged_locations import TaggedLocationStore
from .spatial.topological import TopologicalMemory

__all__ = [
    "EpisodicMemory",
    "EpisodicMemoryModule",
    "TaggedLocationStore",
    "TaggedLocationsModule",
    "TopologicalMemory",
    "TopologicalMemoryModule",
]
