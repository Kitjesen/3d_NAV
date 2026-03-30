"""Memory stack: SemanticMapper + EpisodicMemory + TaggedLocations + VectorMemory."""

from __future__ import annotations

import logging
import os

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)

DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")


def memory(save_dir: str = "", **config) -> Blueprint:
    """Semantic memory: KG + topology + episodes + tagged locations."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    try:
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        bp.add(SemanticMapperModule, save_dir=save_dir)
    except ImportError as e:
        logger.warning("SemanticMapperModule not available: %s", e)

    try:
        from memory.modules.episodic_module import EpisodicMemoryModule
        bp.add(EpisodicMemoryModule)
    except ImportError:
        pass

    try:
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        bp.add(TaggedLocationsModule)
    except ImportError:
        pass

    try:
        from memory.modules.vector_memory_module import VectorMemoryModule
        bp.add(VectorMemoryModule, persist_dir=save_dir)
    except ImportError:
        pass

    try:
        from memory.modules.temporal_memory_module import TemporalMemoryModule
        bp.add(TemporalMemoryModule, save_dir=save_dir)
    except ImportError:
        pass

    return bp
