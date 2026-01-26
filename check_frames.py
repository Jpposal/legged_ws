import pinocchio
import sys

urdf_file = "/tmp/debug_arm1.urdf"

try:
    model = pinocchio.buildModelFromUrdf(urdf_file)
    print("Model built successfully.")
    print(f"Model name: {model.name}")
    print(f"Number of frames: {model.nframes}")
    
    names = [f.name for f in model.frames]
    
    from collections import Counter
    counts = Counter(names)
    duplicates = [name for name, count in counts.items() if count > 1]
    
    if duplicates:
        print(f"DUPLICATE FRAMES FOUND: {duplicates}")
        for dup in duplicates:
            print(f" - Frames with name '{dup}':")
            for i, f in enumerate(model.frames):
                if f.name == dup:
                    print(f"   - Index {i}, Type {f.type}")
    else:
        print("No duplicate frame names found.")

except Exception as e:
    print(f"Error building model: {e}")
