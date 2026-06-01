"""Re-exported from memory.spatial.episodic. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.spatial.episodic')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.spatial.episodic')
    return dir(mod)
