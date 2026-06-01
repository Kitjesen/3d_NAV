"""Re-exported from memory.knowledge.semantic_prior. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.knowledge.semantic_prior')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.knowledge.semantic_prior')
    return dir(mod)
