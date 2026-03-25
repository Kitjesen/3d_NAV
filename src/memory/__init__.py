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
from .spatial.topological import TopologicalMemory
from .spatial.episodic import EpisodicMemory
from .spatial.tagged_locations import TaggedLocationStore
from .modules.topological_module import TopologicalMemoryModule
from .modules.episodic_module import EpisodicMemoryModule
from .modules.tagged_locations_module import TaggedLocationsModule

__all__ = [
    "TopologicalMemory", "EpisodicMemory", "TaggedLocationStore",
    "TopologicalMemoryModule", "EpisodicMemoryModule", "TaggedLocationsModule",
]
