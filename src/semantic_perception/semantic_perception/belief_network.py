"""Re-exported from memory.knowledge.belief.network. Import from there directly."""
from memory.knowledge.belief.network import *  # noqa: F401,F403

# Conditional re-export: BeliefPredictor only exists when torch is available
try:
    from memory.knowledge.belief.network import (  # noqa: F401
        BeliefPredictor,
        build_object_vocabulary,
        build_cooccurrence_matrix,
        build_safety_vector,
        build_safety_loss_weights,
        build_affordance_vectors,
        build_dangerous_mask,
        build_room_prior_vectors,
    )
except ImportError:
    pass
