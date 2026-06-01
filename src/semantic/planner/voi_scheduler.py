"""Re-exported from memory.scheduling.voi_scheduler. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.scheduling.voi_scheduler')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.scheduling.voi_scheduler')
    return dir(mod)
