from pxr import Usd

def read_usd(file_path):
    # Open the USD stage
    stage = Usd.Stage.Open(file_path)
    if not stage:
        print("Failed to open USD file.")
        return

    print(stage.GetPrototypes())
    print(list(stage.Traverse()))
    
    # # Print the stage's root layer path
    # print(f"Opened USD file: {file_path}")
    
    # # Iterate through the stage hierarchy
    # def print_prim_hierarchy(prim, indent=0):
    #     print("  " * indent + prim.GetName())
    #     for child in prim.GetChildren():
    #         print_prim_hierarchy(child, indent + 1)
    
    # # Start from the pseudo-root
    # print_prim_hierarchy(stage.GetPseudoRoot())

# Example usage
if __name__ == "__main__":
    usd_file = "/home/namiko/work/Honda_isaac/IsaacLab/franka_allegro_fixedfinger_3.usd" 
    read_usd(usd_file)
