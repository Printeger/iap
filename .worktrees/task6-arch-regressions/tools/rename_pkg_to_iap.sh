#!/usr/bin/env bash
set -euo pipefail

OLD="${1:-glim}"
NEW="${2:-iap}"

echo "[rename] OLD=$OLD NEW=$NEW"

# 1) package.xml
if [[ -f package.xml ]]; then
  sed -i "s/<name>${OLD}<\/name>/<name>${NEW}<\/name>/g" package.xml
fi

# 2) CMakeLists.txt project()
if [[ -f CMakeLists.txt ]]; then
  sed -i "s/^project(${OLD})/project(${NEW})/g" CMakeLists.txt
  sed -i "s/^project(${OLD} /project(${NEW} /g" CMakeLists.txt
fi

# 3) ros2 launch / resource / ament index / config references
# 只替换“包名级引用”，避免把命名空间/库名全替掉
grep -RIl --exclude-dir=build --exclude-dir=install --exclude-dir=log \
  "package://${OLD}\\b\\|${OLD}/" . | while read -r f; do
    sed -i "s/package:\\/\\/${OLD}\\b/package:\\/\\/${NEW}/g" "$f"
    sed -i "s/\\b${OLD}\\//${NEW}\\//g" "$f"
  done

echo "[rename] Done. Now run: colcon build --packages-select ${NEW}"