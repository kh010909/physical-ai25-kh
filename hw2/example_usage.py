"""
Example: Using the Coordinate Mapper for Part 3

This example demonstrates how to use the coordinate utilities
for navigation and object localization in Part 3.
"""

import numpy as np
from coordinate_utils import create_mapper_from_file, CoordinateMapper


def example_basic_conversion():
    """
    Example 1: Basic coordinate conversion
    """
    print("="*60)
    print("EXAMPLE 1: Basic Coordinate Conversion")
    print("="*60)
    
    try:
        # Load the mapper from saved mapping info
        mapper = create_mapper_from_file(
            filepath='semantic_3d_pointcloud/mapping_info.npy',
            map_width_pixels=1000
        )
        
        # Example: Agent is at position (2.5, 3.0) in Habitat coordinates
        agent_x_habitat = 2.5
        agent_z_habitat = 3.0
        
        # Convert to map pixel coordinates
        agent_x_pixel, agent_z_pixel = mapper.habitat_to_pixel(
            agent_x_habitat, agent_z_habitat
        )
        
        print(f"\n📍 Agent Position:")
        print(f"   Habitat coords: ({agent_x_habitat:.2f}, {agent_z_habitat:.2f}) m")
        print(f"   Map pixel coords: ({agent_x_pixel}, {agent_z_pixel}) px")
        
        # Convert back to verify
        x_back, z_back = mapper.pixel_to_habitat(agent_x_pixel, agent_z_pixel)
        print(f"   Verification: ({x_back:.2f}, {z_back:.2f}) m")
        
        # Check if position is valid
        is_valid = mapper.is_valid_habitat(agent_x_habitat, agent_z_habitat)
        print(f"   Valid position: {is_valid}")
        
    except FileNotFoundError:
        print("\n❌ Error: mapping_info.npy not found!")
        print("   Please run semantic_map.py first to generate the mapping info.")


def example_navigation_path():
    """
    Example 2: Converting a navigation path
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: Navigation Path Conversion")
    print("="*60)
    
    try:
        mapper = create_mapper_from_file(
            filepath='semantic_3d_pointcloud/mapping_info.npy',
            map_width_pixels=1000
        )
        
        # Example path in Habitat coordinates (meters)
        path_habitat = [
            (0.0, 0.0),
            (1.0, 2.0),
            (2.5, 3.5),
            (4.0, 5.0),
        ]
        
        print("\n🛤️  Navigation Path:")
        print("Habitat (m) → Map Pixel (px)")
        print("-" * 40)
        
        path_pixels = []
        for i, (x_h, z_h) in enumerate(path_habitat):
            x_p, z_p = mapper.habitat_to_pixel(x_h, z_h)
            path_pixels.append((x_p, z_p))
            print(f"  Point {i+1}: ({x_h:5.2f}, {z_h:5.2f}) → ({x_p:4d}, {z_p:4d})")
        
        # Calculate path length in pixels
        total_length = 0
        for i in range(len(path_pixels) - 1):
            dx = path_pixels[i+1][0] - path_pixels[i][0]
            dz = path_pixels[i+1][1] - path_pixels[i][1]
            length = np.sqrt(dx**2 + dz**2)
            total_length += length
        
        print(f"\n  Total path length: {total_length:.1f} pixels")
        
    except FileNotFoundError:
        print("\n❌ Error: mapping_info.npy not found!")


def example_object_localization():
    """
    Example 3: Locating objects on the map
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: Object Localization")
    print("="*60)
    
    try:
        mapper = create_mapper_from_file(
            filepath='semantic_3d_pointcloud/mapping_info.npy',
            map_width_pixels=1000
        )
        
        # Example: Objects detected at these Habitat coordinates
        objects = {
            'chair': (1.5, 2.0),
            'table': (3.0, 4.5),
            'door': (0.5, 6.0),
        }
        
        print("\n🎯 Object Locations:")
        print("Object      | Habitat (m)    | Map Pixel (px)   | Valid")
        print("-" * 60)
        
        for obj_name, (x_h, z_h) in objects.items():
            x_p, z_p = mapper.habitat_to_pixel(x_h, z_h)
            is_valid = mapper.is_valid_habitat(x_h, z_h)
            status = "✓" if is_valid else "✗"
            print(f"  {obj_name:8s}  | ({x_h:5.2f}, {z_h:5.2f}) | "
                  f"({x_p:4d}, {z_p:4d})    | {status}")
        
    except FileNotFoundError:
        print("\n❌ Error: mapping_info.npy not found!")


def example_distance_calculation():
    """
    Example 4: Calculate distance between two points
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: Distance Calculation")
    print("="*60)
    
    try:
        mapper = create_mapper_from_file(
            filepath='semantic_3d_pointcloud/mapping_info.npy',
            map_width_pixels=1000
        )
        
        # Two points in Habitat coordinates
        point1_habitat = (1.0, 2.0)
        point2_habitat = (4.0, 6.0)
        
        # Convert to pixels
        p1_pixel = mapper.habitat_to_pixel(*point1_habitat)
        p2_pixel = mapper.habitat_to_pixel(*point2_habitat)
        
        # Calculate distances
        dx_habitat = point2_habitat[0] - point1_habitat[0]
        dz_habitat = point2_habitat[1] - point1_habitat[1]
        distance_habitat = np.sqrt(dx_habitat**2 + dz_habitat**2)
        
        dx_pixel = p2_pixel[0] - p1_pixel[0]
        dz_pixel = p2_pixel[1] - p1_pixel[1]
        distance_pixel = np.sqrt(dx_pixel**2 + dz_pixel**2)
        
        print(f"\n📏 Distance Between Points:")
        print(f"  Point 1: ({point1_habitat[0]:.2f}, {point1_habitat[1]:.2f}) m")
        print(f"  Point 2: ({point2_habitat[0]:.2f}, {point2_habitat[1]:.2f}) m")
        print(f"  Distance in Habitat: {distance_habitat:.2f} m")
        print(f"  Distance in pixels: {distance_pixel:.1f} px")
        print(f"  Scale factor: {mapper.scale:.2f} px/m")
        
    except FileNotFoundError:
        print("\n❌ Error: mapping_info.npy not found!")


def example_custom_mapper():
    """
    Example 5: Creating a custom mapper with different resolution
    """
    print("\n" + "="*60)
    print("EXAMPLE 5: Custom Map Resolution")
    print("="*60)
    
    # Example: Create mappers with different resolutions
    resolutions = [500, 1000, 2000]
    
    print("\n🔍 Comparing Different Map Resolutions:")
    print("Resolution (px) | Scale (px/m) | Map Size (px)")
    print("-" * 50)
    
    # Dummy values for demonstration (replace with actual values)
    x_min, x_max = -5.0, 15.0
    z_min, z_max = -3.0, 17.0
    
    for res in resolutions:
        mapper = CoordinateMapper(x_min, x_max, z_min, z_max, 
                                  map_width_pixels=res)
        print(f"  {res:4d}          | {mapper.scale:8.2f}     | "
              f"{mapper.map_width} x {mapper.map_height}")


def main():
    """
    Run all examples
    """
    print("\n" + "="*70)
    print("COORDINATE MAPPING EXAMPLES FOR PART 3")
    print("="*70)
    print("\nThese examples show how to use coordinate_utils.py for:")
    print("  • Converting between Habitat and pixel coordinates")
    print("  • Planning navigation paths")
    print("  • Localizing objects on the map")
    print("  • Calculating distances")
    print("="*70)
    
    # Run all examples
    example_basic_conversion()
    example_navigation_path()
    example_object_localization()
    example_distance_calculation()
    example_custom_mapper()
    
    print("\n" + "="*70)
    print("💡 TIP: Use these patterns in your Part 3 implementation!")
    print("="*70)
    print("\nKey functions to remember:")
    print("  • create_mapper_from_file() - Load saved mapping")
    print("  • mapper.habitat_to_pixel() - Habitat → Pixel")
    print("  • mapper.pixel_to_habitat() - Pixel → Habitat")
    print("  • mapper.is_valid_habitat() - Check if point is in bounds")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
