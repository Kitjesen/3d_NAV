"""NOVA quadruped dog -- gRPC bridge to brainstem CMS."""

from .connection import NovaDogConnection
from .blueprints import nova_dog_basic, nova_dog_nav, nova_dog_semantic
