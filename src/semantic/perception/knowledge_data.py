"""Re-exported from memory.knowledge.knowledge_data. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.knowledge.knowledge_data')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.knowledge.knowledge_data')
    return dir(mod)
