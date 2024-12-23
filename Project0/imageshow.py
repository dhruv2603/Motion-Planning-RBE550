##This script is only for converting the ppm file to png file for result_demo output visualisation
from PIL import Image

img = Image.open("F:\\WPI\\motionplanning\\code\\build\\result_demo.ppm")
img.save("F:\\WPI\\motionplanning\\results\\custom_path_img_result.png",format='PNG')