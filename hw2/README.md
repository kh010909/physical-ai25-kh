# RRT Pathfinding for 2D Semantic Maps

**NYCU Physical AI 2025 Fall - Homework 2**

A complete implementation of the Rapidly-exploring Random Tree (RRT) algorithm for robot navigation on semantic maps with interactive starting point selection and robot radius consideration.

## Quick Start

```bash
cd src
python3 main_rrt.py
```

## Features

- 🖱️ **Interactive map clicking** to select starting points
- 🤖 **Robot radius consideration** for safe navigation  
- 🎯 **5 target categories**: rack, cushion, sofa, stair, cooktop
- 🌍 **Coordinate transformation** to world coordinate system
- 📊 **Real-time visualization** of exploration tree and paths
- ⚡ **Fast convergence** with goal-biased sampling

## Usage Options

1. **Interactive Mode**: Click on map to select start point
2. **Demo Mode**: Automated pathfinding with predefined points  
3. **Direct Target**: Select specific target category

## Files

- `src/main_rrt.py` - Main application
- `src/rrt_pathfinder.py` - Core RRT implementation
- `RRT_PROJECT_ACHIEVEMENTS.md` - Detailed documentation
- `IMPLEMENTATION_GUIDE.md` - Technical implementation details

## Requirements

- Python 3.7+
- matplotlib, numpy, pandas, openpyxl

## Example Output

The system generates navigable paths with waypoints in both pixel and world coordinates, suitable for robot navigation systems.

See `RRT_PROJECT_ACHIEVEMENTS.md` for complete documentation and technical details.

---
**Original Spec**: https://drive.google.com/file/d/1jg5wRDpTQcx7Ux01hNzPmMdKGN-Mhxc0/view?usp=sharing
