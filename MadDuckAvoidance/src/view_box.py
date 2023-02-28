##visually see if the converted annotations are still a surroundings box for the rubber ducks.
import torch

def box_center_to_corner(boxes):
    """Convert from (center, width, height) to (upper-left, lower-right)."""
    cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = cx - 0.5 * w
    y1 = cy - 0.5 * h
    x2 = cx + 0.5 * w
    y2 = cy + 0.5 * h
    boxes = torch.stack((x1, y1, x2, y2), axis=-1)
    return boxes


from matplotlib import pyplot as plt
from matplotlib import image as mpimg
from matplotlib.patches import Rectangle
from PIL import Image
 
plt.title("Sheep Image")
plt.xlabel("X pixel scaling")
plt.ylabel("Y pixels scaling")
 
image = Image.open("duck_frames/frame_000869.png")

fig, ax = plt.subplots()

box = [0.5945312500000001, 0.4354166666666667, 0.0859375, 0.10416666666666667]

bbox = [box[0] - box[2]/2, box[1] - box[3]/2, box[2], box[3]]


ax.imshow(image)
ax.add_patch(Rectangle((bbox[0]*image.width, bbox[1]*image.height), bbox[2]*image.width, bbox[3]*image.height, linewidth=1, edgecolor='r', facecolor='none'))

plt.show()