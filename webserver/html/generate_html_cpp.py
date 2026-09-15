#!/usr/bin/env python3
import gzip
import os

def generate_cpp_array(html_file, output_file):
    with open(html_file, 'rb') as f:
        html_content = f.read()
    
    compressed = gzip.compress(html_content, compresslevel=9, mtime=0)
    
    base_name = os.path.basename(html_file).replace('.html', '').replace('.', '_')
    array_name = f"{base_name}_html_gz"
    
    with open(output_file, 'w') as f:
        f.write(f"unsigned char {array_name}[] = {{\n")
        
        bytes_per_line = 12
        for i in range(0, len(compressed), bytes_per_line):
            chunk = compressed[i:i+bytes_per_line]
            hex_bytes = ', '.join(f'0x{b:02x}' for b in chunk)
            f.write(f"  {hex_bytes}")
            if i + bytes_per_line < len(compressed):
                f.write(",")
            f.write("\n")
        
        f.write("};\n")
        f.write(f"unsigned int {array_name}_len = {len(compressed)};\n")
    
    print(f"Generated {output_file} ({len(compressed)} bytes from {len(html_content)} bytes)")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.join(os.path.dirname(script_dir), 'src')
    
    html_files = [
        ('index_gc0308_face.html', 'index_gc0308_face_html.cpp')
    ]
    
    for html_file, cpp_file in html_files:
        html_path = os.path.join(script_dir, html_file)
        cpp_path = os.path.join(src_dir, cpp_file)
        
        if os.path.exists(html_path):
            generate_cpp_array(html_path, cpp_path)
        else:
            print(f"Warning: {html_path} not found")
