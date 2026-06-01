"""Re-exported from memory.spatial.topological. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.spatial.topological')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.spatial.topological')
    return dir(mod)
