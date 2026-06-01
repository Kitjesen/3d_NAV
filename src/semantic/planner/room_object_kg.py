"""Re-exported from memory.knowledge.room_object_kg. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.knowledge.room_object_kg')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.knowledge.room_object_kg')
    return dir(mod)
