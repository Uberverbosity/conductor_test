#!/usr/bin/env python3
"""
Convert an image file to LVGL-compatible C array in RGB565 format.
Usage: python img_to_c.py input_image.png output.c [array_name]
"""

import sys
from PIL import Image

def rgb888_to_rgb565(r, g, b):
    """Convert RGB888 to RGB565 format."""
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)

def image_to_c_array(image_path, output_path, array_name="img_data"):
    """Convert image to C array in RGB565 format."""
    try:
        # Open and convert image
        img = Image.open(image_path)
        
        # Ensure image is RGB mode (convert if necessary)
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        # Resize to 240x240 if needed
        if img.size != (240, 240):
            print(f"Warning: Image is {img.size}, resizing to 240x240")
            img = img.resize((240, 240), Image.Resampling.LANCZOS)
        
        width, height = img.size
        
        # Get pixel data
        pixels = img.load()
        
        # Generate C array
        with open(output_path, 'w') as f:
            f.write('#include "lvgl.h"\n\n')
            f.write('#ifndef LV_ATTRIBUTE_MEM_ALIGN\n')
            f.write('#define LV_ATTRIBUTE_MEM_ALIGN\n')
            f.write('#endif\n\n')
            
            # Variable name for the pixel data
            pixel_array_name = f"{array_name}_map"
            descriptor_name = array_name
            
            f.write(f'const LV_ATTRIBUTE_MEM_ALIGN uint8_t {pixel_array_name}[] = {{\n')
            
            # Write pixel data (RGB565 is 2 bytes per pixel)
            for y in range(height):
                f.write('    ')
                for x in range(width):
                    r, g, b = pixels[x, y]
                    rgb565 = rgb888_to_rgb565(r, g, b)
                    
                    # Write as little-endian (LSB first, then MSB)
                    f.write(f'0x{(rgb565 & 0xFF):02X}, 0x{((rgb565 >> 8) & 0xFF):02X}')
                    
                    if not (y == height - 1 and x == width - 1):
                        f.write(', ')
                
                if y < height - 1:
                    f.write('\n')
            
            f.write('\n};\n\n')
            
            # Write image descriptor
            f.write(f'const lv_img_dsc_t {descriptor_name} = {{\n')
            f.write(f'    .header.w = {width},\n')
            f.write(f'    .header.h = {height},\n')
            f.write(f'    .header.cf = LV_COLOR_FORMAT_RGB565,\n')
            f.write(f'    .data_size = {width * height * 2},\n')
            f.write(f'    .data = {pixel_array_name},\n')
            f.write('};\n')
        
        print(f"Successfully converted {image_path} to {output_path}")
        print(f"Image size: {width}x{height}, Array name: {descriptor_name}")
        print(f"Data size: {width * height * 2} bytes")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python img_to_c.py <input_image> <output.c> [array_name]")
        print("Example: python img_to_c.py gradient.png gradient_bg.c gradient_bg")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    array_name = sys.argv[3] if len(sys.argv) > 3 else "img_data"
    
    image_to_c_array(input_file, output_file, array_name)
