"""Re-exported from memory.knowledge.knowledge_graph. Import from there directly."""

def __getattr__(name):
    import importlib
    mod = importlib.import_module('memory.knowledge.knowledge_graph')
    result = getattr(mod, name)
    globals()[name] = result
    return result


def __dir__():
    import importlib
    mod = importlib.import_module('memory.knowledge.knowledge_graph')
    return dir(mod)
