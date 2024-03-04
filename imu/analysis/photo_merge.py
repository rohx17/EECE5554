from PIL import Image

# Open the PNG files
img1 = Image.open("/home/rohit/EECE5554/Files/allan/X-axis_Gyro_Allan_Deviation_Noise_parameters.png")
img2 = Image.open("/home/rohit/EECE5554/Files/allan/Y-axis_Gyro_Allan_Deviation_Noise_parameters.png")
img3 = Image.open("/home/rohit/EECE5554/Files/allan/Z-axis_Gyro_Allan_Deviation_Noise_parameters.png")

# Get the dimensions of the images
width1, height1 = img1.size
width2, height2 = img2.size
width3, height3 = img3.size

# Create a new image with the combined width and the maximum height
max_width = max(width1, width2, width3)
new_img = Image.new("RGB", (max_width, height1 + height2 + height3))

new_img.paste(img1, (0, 0))
new_img.paste(img2, (0, height1))
new_img.paste(img3, (0, height1 + height2))
# Save the merged image
new_img.save("/home/rohit/EECE5554/Files/allan/Gyro_Allan_Deviation_Noise_parameters.png")

print("Images merged successfully.")