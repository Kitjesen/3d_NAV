"""NOVA quadruped dog -- gRPC bridge to brainstem CMS."""

from .han_dog_module import ThunderDriver
from .connection import NovaDogConnection  # deprecated, use ThunderDriver
from .blueprints import nova_dog_basic, nova_dog_nav, nova_dog_semantic
