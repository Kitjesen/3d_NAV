"""Re-exported from memory.spatial.tagged_locations. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.spatial.tagged_locations')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.spatial.tagged_locations')
    return dir(mod)
