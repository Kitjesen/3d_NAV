"""NOVA quadruped dog -- gRPC bridge to brainstem CMS."""

from .blueprints import nova_dog_basic, nova_dog_nav, nova_dog_semantic
from .connection import NovaDogConnection  # deprecated, use ThunderDriver
from .han_dog_module import ThunderDriver
