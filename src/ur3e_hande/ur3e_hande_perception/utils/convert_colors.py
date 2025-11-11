import json
from perc_utils import hex_to_rgb, COLOR_JSON, COLOR_RGB_JSON

# Load the input JSON file
with open(COLOR_JSON, 'r') as f:
    colors = json.load(f)

# Add RGB values to each color entry
for color in colors:
    rgb = hex_to_rgb(color['hex'])
    color['rgb'] = rgb

# Save to a new JSON file
with open(COLOR_RGB_JSON, 'w') as f:
    json.dump(colors, f, indent=4)