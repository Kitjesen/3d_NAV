"""VectorMemoryModule — CLIP embedding + ChromaDB vector search for fuzzy spatial queries.

Stores scene snapshots as CLIP embeddings associated with robot position.
Supports queries like "去上次放背包的地方" → vector search → return position.

Pipeline:
  scene_graph + odometry + image → CLIP encode scene labels → ChromaDB upsert
  query text → CLIP encode → ChromaDB search → top-k (position, score, labels)

ChromaDB runs in-process (persistent mode, no server needed).
Falls back to brute-force numpy search if chromadb not installed.

Ports:
  In:  scene_graph (SceneGraph), odometry (Odometry), image (np.ndarray)
  Out: query_result (dict)  — triggered by query_location skill
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.semantic import SceneGraph
from core.msgs.nav import Odometry
from core.registry import register

logger = logging.getLogger(__name__)

_DEFAULT_PERSIST_DIR = os.path.join(os.path.expanduser("~"), ".nova", "vector_memory")


@register("vector_memory", "default", description="CLIP + ChromaDB spatial vector search")
class VectorMemoryModule(Module, layer=3):
    """Vector-based spatial memory for fuzzy location queries.

    Stores CLIP embeddings of scene snapshots indexed by robot position.
    Queries return the closest matching location from past experience.
    """

    scene_graph: In[SceneGraph]
    odometry:    In[Odometry]
    image:       In[np.ndarray]

    query_result: Out[dict]

    def __init__(
        self,
        persist_dir: str = "",
        collection_name: str = "spatial_memory",
        store_interval: float = 5.0,
        max_results: int = 5,
        **kw,
    ):
        super().__init__(**kw)
        self._persist_dir = persist_dir or _DEFAULT_PERSIST_DIR
        self._collection_name = collection_name
        self._store_interval = store_interval
        self._max_results = max_results

        self._collection = None  # ChromaDB collection
        self._encoder = None     # CLIP text/image encoder
        self._use_chromadb = False

        # Fallback numpy store
        self._np_embeddings: List[np.ndarray] = []
        self._np_metadata: List[dict] = []

        self._robot_xy = (0.0, 0.0)
        self._latest_labels: List[str] = []
        self._latest_image: Optional[np.ndarray] = None
        self._last_store_time = 0.0
        self._store_count = 0

    def setup(self) -> None:
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        self.image.subscribe(self._on_image)
        self.scene_graph.set_policy("throttle", interval=self._store_interval)

        self._init_encoder()
        self._init_store()

    def _init_encoder(self) -> None:
        try:
            from semantic.perception.semantic_perception.clip_encoder import CLIPEncoder
            self._encoder = CLIPEncoder(model_name="mobileclip")
            logger.info("VectorMemoryModule: CLIP encoder ready")
        except ImportError:
            logger.warning("VectorMemoryModule: CLIP encoder not available, text-only mode")

    def _init_store(self) -> None:
        try:
            import chromadb
            os.makedirs(self._persist_dir, exist_ok=True)
            client = chromadb.PersistentClient(path=self._persist_dir)
            self._collection = client.get_or_create_collection(
                name=self._collection_name,
                metadata={"hnsw:space": "cosine"},
            )
            self._use_chromadb = True
            count = self._collection.count()
            logger.info("VectorMemoryModule: ChromaDB ready (%d entries)", count)
        except ImportError:
            logger.info("VectorMemoryModule: chromadb not installed, using numpy fallback")
        except Exception as e:
            logger.warning("VectorMemoryModule: ChromaDB init failed (%s), using numpy", e)

    # ── Input handlers ────────────────────────────────────────────────────────

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy = (odom.x, odom.y)

    def _on_image(self, img: np.ndarray) -> None:
        self._latest_image = img

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        labels = [obj.label for obj in sg.objects if obj.label]
        if not labels:
            return

        self._latest_labels = labels
        now = time.time()
        if now - self._last_store_time < self._store_interval:
            return

        self._store_snapshot(labels)
        self._last_store_time = now

    # ── Store ─────────────────────────────────────────────────────────────────

    def _store_snapshot(self, labels: List[str]) -> None:
        text = ", ".join(sorted(set(labels)))
        embedding = self._encode_text(text)
        if embedding is None:
            return

        meta = {
            "x": float(self._robot_xy[0]),
            "y": float(self._robot_xy[1]),
            "labels": text,
            "ts": time.time(),
        }
        doc_id = f"snap_{self._store_count}"
        self._store_count += 1

        if self._use_chromadb and self._collection is not None:
            self._collection.upsert(
                ids=[doc_id],
                embeddings=[embedding.tolist()],
                metadatas=[meta],
                documents=[text],
            )
        else:
            self._np_embeddings.append(embedding)
            self._np_metadata.append(meta)

    def _encode_text(self, text: str) -> Optional[np.ndarray]:
        if self._encoder is not None and hasattr(self._encoder, "encode_text"):
            try:
                results = self._encoder.encode_text([text])
                if results is not None and len(results) > 0:
                    vec = np.array(results[0], dtype=np.float32).flatten()
                    norm = float(np.linalg.norm(vec))
                    if norm > 1e-8:
                        vec /= norm
                        return vec
            except Exception as e:
                logger.debug("VectorMemory: encode_text failed: %s", e)
        # Fallback: simple bag-of-words hash embedding
        return self._hash_embedding(text)

    @staticmethod
    def _hash_embedding(text: str, dim: int = 512) -> np.ndarray:
        """Deterministic hash embedding for text (fallback when no CLIP).

        Tokenizes on spaces, commas, and common stop words to ensure
        'find backpack' matches 'backpack, bench'.
        """
        import re
        _STOP = {"go", "to", "the", "find", "where", "is", "a", "an", "my", "at", "in", "on", "for", "of", "area", "place", "spot"}
        words = re.split(r'[,\s]+', text.lower().strip())
        words = [w for w in words if w and w not in _STOP]
        vec = np.zeros(dim, dtype=np.float32)
        for word in words:
            h = hash(word) % dim
            vec[h] += 1.0
        norm = np.linalg.norm(vec)
        if norm > 0:
            vec /= norm
        return vec

    # ── Query ─────────────────────────────────────────────────────────────────

    def _query(self, text: str, n: int = 0) -> List[dict]:
        n = n or self._max_results
        embedding = self._encode_text(text)
        if embedding is None:
            return []

        if self._use_chromadb and self._collection is not None:
            try:
                results = self._collection.query(
                    query_embeddings=[embedding.tolist()],
                    n_results=min(n, max(self._collection.count(), 1)),
                )
                hits = []
                for i in range(len(results["ids"][0])):
                    meta = results["metadatas"][0][i]
                    hits.append({
                        "x": meta["x"],
                        "y": meta["y"],
                        "labels": meta.get("labels", ""),
                        "score": 1.0 - results["distances"][0][i],  # cosine → similarity
                        "ts": meta.get("ts", 0),
                    })
                return hits
            except Exception as e:
                logger.warning("VectorMemory: ChromaDB query failed: %s", e)
                return []
        else:
            return self._numpy_query(embedding, n)

    def _numpy_query(self, query_vec: np.ndarray, n: int) -> List[dict]:
        if not self._np_embeddings:
            return []
        mat = np.stack(self._np_embeddings)
        sims = mat @ query_vec
        top_idx = np.argsort(sims)[::-1][:n]
        return [
            {**self._np_metadata[i], "score": float(sims[i])}
            for i in top_idx if sims[i] > 0.1
        ]

    # ── @skill methods (MCP-exposed) ──────────────────────────────────────────

    @skill
    def query_location(self, text: str) -> dict:
        """Fuzzy search for a location by natural language description.

        Example: "去上次放背包的地方" → returns closest matching position.
        """
        results = self._query(text)
        if not results:
            return {"query": text, "found": False, "results": []}
        return {
            "query": text,
            "found": True,
            "best": {"x": results[0]["x"], "y": results[0]["y"],
                     "labels": results[0]["labels"], "score": results[0]["score"]},
            "results": results[:3],
        }

    @skill
    def get_memory_stats(self) -> dict:
        """Return vector memory statistics."""
        count = 0
        if self._use_chromadb and self._collection is not None:
            count = self._collection.count()
        else:
            count = len(self._np_embeddings)
        return {
            "backend": "chromadb" if self._use_chromadb else "numpy",
            "entries": count,
            "store_count": self._store_count,
            "persist_dir": self._persist_dir,
        }
