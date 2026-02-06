#!/bin/bash
# Run from sfc_planner/docker/ directory

# Get the parent directory (sfc_planner root)
PROJECT_ROOT="$(cd .. && pwd)"

echo "Starting SFC Planner Container..."
echo "Mounting: $PROJECT_ROOT"

docker run -it --rm \
  --name sfc_dev \
  -v "$PROJECT_ROOT:/home/prance/sfc_planner" \
  prance:sfc-planner \
  bash