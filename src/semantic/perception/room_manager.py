"""Re-exported from memory.spatial.room_manager. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.spatial.room_manager')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.spatial.room_manager')
    return dir(mod)
