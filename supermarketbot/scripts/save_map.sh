#!/bin/bash
# Save the current SLAM map to the supermarketbot package maps directory.
# Usage: ./save_map.sh

MAP_DIR="$(dirname "$0")/../maps"
MAP_NAME="world_map"

echo "🗺️  Saving map to ${MAP_DIR}/${MAP_NAME} ..."
ros2 run nav2_map_server map_saver_cli \
    -f "${MAP_DIR}/${MAP_NAME}" \
    --ros-args -p use_sim_time:=true

if [ $? -eq 0 ]; then
    echo "✅ Map saved! Files:"
    ls -lh "${MAP_DIR}/${MAP_NAME}".*
    echo ""
    echo "Rebuild the package to install the new map:"
    echo "  cd ~/s_ws && colcon build --packages-select supermarketbot && source install/setup.bash"
else
    echo "❌ Map save failed. Is SLAM still running?"
fi
