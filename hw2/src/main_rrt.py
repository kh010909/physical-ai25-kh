#!/usr/bin/env python3
"""
RRT Pathfinding Main Application

Complete implementation of RRT pathfinding algorithm for 2D semantic maps.
Implements the Rapidly-exploring Random Tree (RRT) algorithm to calculate 
navigable paths from user-selected starting points to specified target 
object categories, with robot radius consideration and coordinate transformation.
"""

import sys
import os
import matplotlib
from rrt_pathfinder import RRTPathfinder


def main():
    """Main application entry point for RRT pathfinding."""
    print_header()
    
    # Validate required files
    if not validate_files():
        return
    
    # Initialize pathfinder
    pathfinder = initialize_pathfinder()
    if not pathfinder:
        return
    
    # Main application loop
    while True:
        choice = get_user_choice(pathfinder)
        
        if choice == 'quit':
            print("\n👋 Thank you for using RRT Pathfinder!")
            break
        elif choice == 'interactive':
            run_interactive_mode(pathfinder)
        elif choice == 'demo':
            run_demo_mode(pathfinder)
        elif choice in pathfinder.target_categories:
            run_pathfinding(pathfinder, choice)
        else:
            print("❌ Invalid choice. Please try again.")


def print_header():
    """Print application header and description."""
    print("=" * 70)
    print("🤖 RRT PATHFINDING FOR 2D SEMANTIC MAPS")
    print("=" * 70)
    print()
    print("This application implements the Rapidly-exploring Random Tree (RRT)")
    print("algorithm to find navigable paths on semantic maps with:")
    print("  • Interactive starting point selection via map clicking")
    print("  • Robot radius and safety margin consideration")  
    print("  • Obstacle avoidance with collision detection")
    print("  • Coordinate transformation to world coordinate system")
    print("  • Real-time path visualization and exploration tree display")
    print()


def validate_files():
    """Validate that all required files are present."""
    required_files = [
        ('map.png', 'Semantic map image'),
        ('../color_coding_semantic_segmentation_classes.xlsx', 'Color mapping data'),
        ('coordinate_transformation.txt', 'Coordinate transformation parameters')
    ]
    
    missing_files = []
    for file_path, description in required_files:
        if not os.path.exists(file_path):
            missing_files.append((file_path, description))
    
    if missing_files:
        print("❌ ERROR: Missing required files:")
        for file_path, description in missing_files:
            print(f"  • {file_path} - {description}")
        print("\n💡 Please ensure all required files are present and try again.")
        return False
    
    return True


def initialize_pathfinder():
    """Initialize the RRT pathfinder with error handling."""
    try:
        print("🔧 Initializing RRT pathfinder...")
        pathfinder = RRTPathfinder(
            'map.png', 
            '../color_coding_semantic_segmentation_classes.xlsx', 
            'coordinate_transformation.txt'
        )
        
        print("✅ Pathfinder initialized successfully!")
        print(f"   • Step size: {pathfinder.step_size} pixels")
        print(f"   • Robot radius: {pathfinder.robot_radius_pixels} pixels") 
        print(f"   • Safety margin: {pathfinder.safety_margin_pixels} pixels")
        print(f"   • Max iterations: {pathfinder.max_iterations}")
        print()
        
        return pathfinder
        
    except Exception as e:
        print(f"❌ ERROR: Failed to initialize pathfinder: {e}")
        return None


def get_user_choice(pathfinder):
    """Get user's choice for pathfinding mode and target."""
    print("🎯 Available target categories:")
    categories = list(pathfinder.target_categories.keys())
    for i, category in enumerate(categories, 1):
        color = pathfinder.target_categories[category]
        print(f"  {i}. {category.upper()} (RGB: {color})")
    
    print("\n📋 Options:")
    print("  • Enter category name (e.g., 'rack', 'sofa', 'cushion')")
    print("  • Enter number (1-5) for category")
    print("  • 'interactive' - Click on map to select starting point")
    print("  • 'demo' - Run automated demo with predefined points")
    print("  • 'quit' - Exit application")
    
    while True:
        try:
            user_input = input("\n🎮 Your choice: ").strip().lower()
            
            if user_input in ['quit', 'interactive', 'demo']:
                return user_input
            elif user_input.isdigit():
                choice_num = int(user_input)
                if 1 <= choice_num <= len(categories):
                    return categories[choice_num - 1]
                else:
                    print(f"❌ Invalid number! Please enter 1-{len(categories)}")
            elif user_input in categories:
                return user_input
            else:
                print(f"❌ Invalid choice! Available: {', '.join(categories + ['interactive', 'demo', 'quit'])}")
                
        except (ValueError, KeyboardInterrupt):
            return 'quit'


def run_interactive_mode(pathfinder):
    """Run interactive mode with map clicking."""
    print("\n" + "="*60)
    print("🖱️  INTERACTIVE MODE - CLICK TO SELECT START POINT")
    print("="*60)
    
    # Set interactive backend
    matplotlib.use('TkAgg')
    
    # Get target category
    print("\n🎯 Select target category for interactive pathfinding:")
    categories = list(pathfinder.target_categories.keys())
    for i, category in enumerate(categories, 1):
        print(f"  {i}. {category.upper()}")
    
    while True:
        try:
            choice = input("\nEnter category name or number: ").strip().lower()
            if choice.isdigit() and 1 <= int(choice) <= len(categories):
                target_category = categories[int(choice) - 1]
                break
            elif choice in categories:
                target_category = choice
                break
            else:
                print("❌ Invalid choice. Try again.")
        except (ValueError, KeyboardInterrupt):
            print("\n🔄 Returning to main menu...")
            return
    
    try:
        # Run full interactive pathfinding
        print(f"\n🚀 Starting interactive pathfinding to: {target_category.upper()}")
        pixel_path, habitat_coords = pathfinder.run_pathfinding(target_category)
        
        if habitat_coords:
            print("\n🎉 INTERACTIVE PATHFINDING COMPLETED!")
            display_results(pixel_path, habitat_coords, target_category)
        else:
            print("\n❌ Interactive pathfinding failed or was cancelled.")
            
    except Exception as e:
        print(f"\n❌ Error in interactive mode: {e}")


def run_demo_mode(pathfinder):
    """Run automated demo mode with predefined starting points."""
    print("\n" + "="*60)
    print("🤖 AUTOMATED DEMO MODE")
    print("="*60)
    
    demo_configs = [
        {'category': 'rack', 'start': (600, 400), 'desc': 'Storage rack pathfinding'},
        {'category': 'cushion', 'start': (700, 600), 'desc': 'Cushion/seating area'},
        {'category': 'sofa', 'start': (300, 500), 'desc': 'Sofa furniture target'},
        {'category': 'cooktop', 'start': (200, 300), 'desc': 'Kitchen cooktop area'},
    ]
    
    successful_paths = 0
    
    for i, config in enumerate(demo_configs, 1):
        print(f"\n--- Demo {i}: {config['desc']} ---")
        print(f"Target: {config['category'].upper()}")
        print(f"Start: {config['start']}")
        
        try:
            goal_point = pathfinder.find_goal_point(config['category'])
            path = pathfinder.run_rrt(config['start'], goal_point)
            
            if path:
                output_file = f"demo_{config['category']}_path.png"
                pathfinder.visualize_path(output_file)
                habitat_coords = pathfinder.pixel_to_habitat_coordinates(path)
                
                print(f"✅ SUCCESS!")
                print(f"   • Path length: {len(path)} waypoints")
                print(f"   • Tree nodes explored: {len(pathfinder.tree_nodes)}")
                print(f"   • Visualization saved: {output_file}")
                print(f"   • Start (habitat): ({habitat_coords[0][0]:.3f}, {habitat_coords[0][1]:.3f})")
                print(f"   • Goal (habitat): ({habitat_coords[-1][0]:.3f}, {habitat_coords[-1][1]:.3f})")
                
                successful_paths += 1
            else:
                print(f"❌ FAILED: No path found to {config['category']}")
                
        except Exception as e:
            print(f"❌ ERROR: {e}")
    
    print(f"\n🏁 DEMO COMPLETED: {successful_paths}/{len(demo_configs)} paths successful")


def run_pathfinding(pathfinder, target_category):
    """Run pathfinding for specified target with predefined start point."""
    print(f"\n{'='*50}")
    print(f"🎯 PATHFINDING TO: {target_category.upper()}")
    print(f"{'='*50}")
    
    # Predefined starting points for each category
    demo_starts = {
        'sofa': (300, 500),
        'rack': (600, 400), 
        'cushion': (700, 600),
        'stair': (400, 800),
        'cooktop': (200, 300)
    }
    
    start_point = demo_starts.get(target_category, (400, 400))
    
    try:
        print(f"📍 Using start point: {start_point}")
        print("💡 (Use 'interactive' mode to click and select your own start point)")
        
        # Find goal point
        goal_point = pathfinder.find_goal_point(target_category)
        print(f"🎯 Target found at: {goal_point}")
        
        # Run RRT algorithm
        print("🔄 Running RRT algorithm...")
        path = pathfinder.run_rrt(start_point, goal_point)
        
        if path:
            print(f"\n🎉 SUCCESS: Path found with {len(path)} waypoints!")
            
            # Create visualization
            output_file = f"path_to_{target_category}.png"
            pathfinder.visualize_path(output_file)
            
            # Transform coordinates and display results
            habitat_coords = pathfinder.pixel_to_habitat_coordinates(path)
            display_results(path, habitat_coords, target_category, output_file)
            
        else:
            print(f"\n❌ FAILED: No path found to {target_category}!")
            print("💡 Possible solutions:")
            print("   • Try a different starting point (use 'interactive' mode)")
            print("   • Reduce robot radius if it's too large")
            print("   • Increase max iterations for more exploration")
            
    except Exception as e:
        print(f"\n❌ ERROR during pathfinding: {e}")


def display_results(path, habitat_coords, target_category, output_file=None):
    """Display pathfinding results in a formatted manner."""
    print(f"\n📊 PATHFINDING RESULTS - {target_category.upper()}")
    print("="*50)
    
    if output_file:
        print(f"📁 Visualization saved to: {output_file}")
    
    print(f"🛤️  Path Details:")
    print(f"   • Waypoints: {len(path)}")
    print(f"   • Start (habitat): ({habitat_coords[0][0]:.3f}, {habitat_coords[0][1]:.3f})")
    print(f"   • Goal (habitat): ({habitat_coords[-1][0]:.3f}, {habitat_coords[-1][1]:.3f})")
    
    print(f"\n🌍 Habitat Coordinate System Path:")
    print("   Waypoint    X-coord    Z-coord")
    print("   ---------  ---------  ---------")
    
    # Show first 8 and last 8 points for readability
    show_points = min(8, len(habitat_coords))
    for i in range(show_points):
        x, z = habitat_coords[i]
        print(f"      {i:2d}     {x:8.3f}   {z:8.3f}")
    
    if len(habitat_coords) > 16:
        omitted = len(habitat_coords) - 16
        print(f"      ...    ({omitted} middle waypoints omitted)")
        for i in range(len(habitat_coords) - 8, len(habitat_coords)):
            x, z = habitat_coords[i]
            print(f"      {i:2d}     {x:8.3f}   {z:8.3f}")
    elif len(habitat_coords) > show_points:
        for i in range(show_points, len(habitat_coords)):
            x, z = habitat_coords[i]
            print(f"      {i:2d}     {x:8.3f}   {z:8.3f}")
    
    print(f"\n✅ Path ready for robot navigation!")
    print("💡 Use these coordinates to program your robot's movement.")


if __name__ == "__main__":
    main()