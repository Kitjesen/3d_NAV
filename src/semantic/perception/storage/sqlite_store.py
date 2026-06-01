"""Re-exported from memory.storage.sqlite_store. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.storage.sqlite_store')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.storage.sqlite_store')
    return dir(mod)
