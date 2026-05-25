import octo
from octo.model.octo_model import OctoModel

print("Loading Octo model...")
model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
print("Model loaded successfully!")
print(f"Model config keys: {list(model.config.keys())}")
