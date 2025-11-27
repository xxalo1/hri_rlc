# save as convert_textures.py and run with your cs285 env
from PIL import Image

Image.open(r"world/textures/mossy_cobblestone_diff_8k.jpg") \
     .save(r"world/textures/mossy_cobblestone_diff_8k.png")
print("Done.")
