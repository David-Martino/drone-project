import importlib, importlib.metadata, cflib
print("cflib file:", getattr(cflib, "__file__", "n/a"))
try:
    print("cflib version:", importlib.metadata.version("cflib"))
except Exception as e:
    print("cflib version: <unknown>", e)
